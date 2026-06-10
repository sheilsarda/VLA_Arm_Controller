# Plan: Integrate Gemini ER into VLA_Arm_Controller

## Context

**vla-pipeline** demonstrates that you can get a modular, retrain-free robotics pipeline by using Gemini ER for perception + task reasoning and classical code for kinematics/control. Currently it runs in a browser controlling a WidowX arm in MuJoCo WASM.

**VLA_Arm_Controller** controls a UR5e arm in Isaac Sim via ROS2, currently using an end-to-end pi0-FAST model (OpenPI). The goal is to replace the OpenPI inference with the Gemini ER modular approach — keeping the existing ROS2/Isaac Sim/MoveIt infrastructure.

The key architectural shift: OpenPI runs a continuous inference loop (observe → infer deltas → execute → repeat every ~1s). Gemini ER is one-shot: capture image → detect objects → plan task → convert to 3D waypoints → execute all waypoints sequentially. This requires a different execution model, not just swapping the inference call.

---

## Approach: New node + MoveIt for IK/collision avoidance

Create a **new** `gemini_er_controller_node` alongside the existing `vla_controller_node` (doesn't break the OpenPI workflow). Use **MoveIt** (already configured for UR5e) for inverse kinematics and collision-aware planning, replacing the custom Jacobian IK and potential field from vla-pipeline.

### Backwards compatibility

All existing workflows continue to work unchanged:

- **pi0-FAST inference via OpenPI**: `vla_controller_node.py` is **not modified**. `vla_system.launch.py` is **not modified**. Running `ros2 launch vla_controller vla_system.launch.py task:="..." ` works exactly as before.
- **MoveIt demo / manual control**: `controller_v1.launch.py` is **not modified**. MoveIt configuration files (kinematics.yaml, joint_limits.yaml, SRDF, etc.) are **not modified**.
- **Eval recording**: `eval_recorder.py` is **not modified**.

The only changes to existing files are **additive**:
- `setup.py`: adds a new entry point and new data_files entries (existing entries untouched)
- `package.xml`: adds new `<exec_depend>` lines (existing deps untouched)

The two modes are launched independently — you either run `vla_system.launch.py` (OpenPI) or `gemini_er_system.launch.py` (Gemini ER), never both.

---

## New files to create

All under `src/vla_controller/` in VLA_Arm_Controller.

### 1. `vla_controller/gemini_er_client.py` — Gemini API client

Port of `vla-pipeline/web/gemini-er.js`. Uses `google-genai` Python SDK.

- `detect_objects(client, image_bytes)` — sends detection prompt (port of lines 178-192 of gemini-er.js), returns list of `{label, point, box_2d, type}`
- `generate_plan(client, image_bytes, detections, task)` — sends planning prompt (port of lines 221-241), returns list of `{function, args}`
- `plan_to_waypoints(plan_steps, cam_intrinsics, cam_extrinsics, table_z, grasp_height, transit_height)` — port of `planToWaypoints()` (lines 289-330), converts plan steps to 3D `[{xyz, gripper_open, is_gripper_change}]`
- `detect_and_plan(client, image_bytes, task, cam_intrinsics, cam_extrinsics, table_z, ...)` — orchestrator combining the above
- JSON parsing with markdown fence extraction (port of lines 122-131)
- Constants `HEIGHT_OFFSET`, `GRASP_HEIGHT`, `TABLE_Z` become function parameters (configurable via ROS params)

### 2. `vla_controller/camera_geometry.py` — Pixel-to-3D projection

Port of `vla-pipeline/web/math-utils.js` lines 253-316. Pure NumPy, no ROS dependencies.

- `pixel_to_ray(px, py, K, cam_pos, cam_rot)` — port of `pixelToRay()`, but uses a camera intrinsics matrix `K` (from `/camera/camera_info`) instead of deriving from FOV. Returns ray origin + direction in world frame.
- `pixel_to_world_3d(px, py, K, cam_pos, cam_rot, table_z)` — port of `pixelToWorld3d()`. Ray-plane intersection at z=table_z.
- `bbox_to_obstacle_3d(bbox, point, K, cam_pos, cam_rot, depth_image=None, table_z=0.0)` — port of `bboxToObstacle3d()` (gemini-er.js lines 33-61). Projects bbox edges to 3D. If depth_image available, samples depth at center; otherwise falls back to table_z plane intersection.

Key adaptation: Isaac Sim cameras use ROS conventions (+Z forward), not OpenGL (-Z forward). The `camera_info` K matrix handles projection math; extrinsics come from TF2 (`world` → `camera_optical_frame`).

### 3. `vla_controller/gemini_er_controller_node.py` — Main ROS2 node

State machine: `IDLE → CAPTURING → DETECTING → PLANNING → EXECUTING → DONE`

**Subscriptions** (reuses same Isaac Sim topics):
- `/camera/image_raw` — primary scene camera
- `/camera/camera_info` — intrinsics (K matrix)
- `/joint_states` — current arm state

**Action clients** (reuses existing controller infrastructure):
- `/ur_manipulator_controller/follow_joint_trajectory` — arm motion
- `/gripper_controller/follow_joint_trajectory` — gripper open/close

**MoveIt interfaces**:
- `MoveGroupCommander("ur_manipulator")` — IK + collision-aware motion planning
- `PlanningSceneInterface()` — inject detected obstacles as collision objects

**Service** (trigger execution):
- `~/trigger` (`std_srvs/Trigger`) — start a detect-plan-execute cycle

**Flow per cycle:**
1. **CAPTURING**: Grab one frame from `/camera/image_raw` + camera_info. Look up camera extrinsics via TF2 (`world` → `camera_optical_frame`).
2. **DETECTING**: Call `detect_objects()` via Gemini API (in background thread to avoid blocking ROS executor). Parse detections.
3. **PLANNING**: Call `generate_plan()` via Gemini API. Convert plan to 3D waypoints via `plan_to_waypoints()`. For each obstacle detection, call `bbox_to_obstacle_3d()` and add as a MoveIt collision object.
4. **EXECUTING**: For each waypoint sequentially:
   - **Move waypoint**: Set MoveIt pose target (xyz from waypoint + fixed downward-facing orientation for tool0). Plan + execute. The orientation quaternion for "tool0 pointing down" needs to be calibrated for the UR5e (approximately `qx=0, qy=0.707, qz=0, qw=0.707`).
   - **Gripper waypoint**: Send single-point trajectory to gripper action server (0.0=open, 0.8=closed for Robotiq 85). Wait for completion.
5. **DONE**: Log results, clear planning scene obstacles, return to IDLE.

**Dry-run mode**: Runs through CAPTURING → DETECTING → PLANNING, logs waypoints, but skips EXECUTING.

### 4. `config/gemini_er_params.yaml`

```yaml
gemini_er_controller_node:
  ros__parameters:
    base_camera_topic: "/camera/image_raw"
    camera_info_topic: "/camera/camera_info"
    task_instruction: "pick up the red block and place it on the blue target"
    table_z: 0.0
    grasp_height: 0.04
    transit_height: 0.15
    gripper_open_position: 0.0
    gripper_close_position: 0.8
    planning_group: "ur_manipulator"
    ee_link: "tool0"
    planning_time: 5.0
    max_velocity_scaling: 0.1
    max_acceleration_scaling: 0.1
    auto_start: false
    dry_run: false
    arm_action_name: "/ur_manipulator_controller/follow_joint_trajectory"
    gripper_action_name: "/gripper_controller/follow_joint_trajectory"
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
                  wrist_1_joint, wrist_2_joint, wrist_3_joint]
    gripper_joint: "robotiq_85_left_knuckle_joint"
```

### 5. `launch/gemini_er_system.launch.py`

Mirrors `vla_system.launch.py` structure but:
- Launches `gemini_er_controller_node` instead of `vla_controller_node`
- Adds `move_group` node (required for MoveIt Python API — pattern from `controller_v1.launch.py` lines 62-70)
- No OpenPI/venv PYTHONPATH injection needed
- Injects `google-genai` venv path instead (or relies on system install)
- Launch args: `task` (required), `dry_run`, `auto_start`, `table_z`

---

## Existing files to modify

### 6. `src/vla_controller/setup.py`

- Add entry point: `"gemini_er_controller_node = vla_controller.gemini_er_controller_node:main"`
- Add to data_files: `config/gemini_er_params.yaml`, `launch/gemini_er_system.launch.py`

### 7. `src/vla_controller/package.xml`

Add exec_depends: `moveit_ros_planning_interface`, `moveit_msgs`, `geometry_msgs`, `visualization_msgs`, `std_srvs`, `tf2_ros`

---

## What gets ported vs. replaced vs. reused

| Component | vla-pipeline source | ROS2 approach |
|-----------|-------------------|---------------|
| Gemini API calls | `gemini-er.js` prompts | Port prompts verbatim to Python |
| JSON parsing | `gemini-er.js` parseJson | Port fence extraction to Python |
| Pixel-to-3D | `math-utils.js` pixelToRay/pixelToWorld3d | Port to Python, use K matrix from camera_info instead of FOV |
| Plan-to-waypoints | `gemini-er.js` planToWaypoints | Port to Python |
| Obstacle extraction | `gemini-er.js` bboxToObstacle3d | Port to Python → MoveIt collision objects |
| IK solver | `controller.js` pseudoinverse Jacobian | **Replace** with MoveIt KDL IK (already configured) |
| Obstacle avoidance | `controller.js` potential field | **Replace** with MoveIt planning scene |
| Trajectory execution | `main.js` animation loop | **Replace** with FollowJointTrajectory action server (already exists) |
| Camera intrinsics | `math-utils.js` from FOV | Read from `/camera/camera_info` topic |
| Camera extrinsics | MuJoCo `cam_xpos`/`cam_xmat` | TF2 lookup (`world` → `camera_optical_frame`) |

---

## Incremental testing plan

### Phase 1: Camera geometry (no Gemini needed)
1. Create `camera_geometry.py` with unit tests using known inputs/outputs
2. Launch Isaac Sim + controller stack
3. Verify `ros2 topic echo /camera/camera_info` provides K matrix
4. Verify `ros2 run tf2_ros tf2_echo world camera_optical_frame` provides extrinsics
5. Place a known object, manually supply pixel coords, verify `pixel_to_world_3d()` returns correct world position

### Phase 2: Gemini API client (standalone, no ROS)
1. Create `gemini_er_client.py`
2. Save a frame from Isaac Sim: `ros2 run image_tools cam2image`
3. Run standalone test script: send to Gemini, verify detections and plan look reasonable

### Phase 3: MoveIt integration (no Gemini needed)
1. Launch full stack with `controller_v1.launch.py` (includes move_group)
2. Test script: use `moveit_commander` to plan + execute a Cartesian pose target, verify UR5e moves in Isaac Sim
3. Test adding a collision object to planning scene and verify planner avoids it
4. Test gripper open/close via gripper action server

### Phase 4: Dry-run integration
1. Create `gemini_er_controller_node.py` + launch file
2. Launch with `dry_run:=true`, trigger via service call
3. Verify state machine progresses, Gemini calls succeed, waypoints are logged
4. Verify obstacle collision objects appear in RViz planning scene

### Phase 5: Full execution
1. Launch with `dry_run:=false`
2. Start with simple task: "pick up the red block"
3. Calibrate: `table_z`, `grasp_height`, `transit_height`, tool0 grasp orientation
4. Iterate on MoveIt planning params (velocity scaling, planning time)

---

## Key risks

1. **Camera frame conventions**: Isaac Sim uses ROS conventions (+Z forward), vla-pipeline uses OpenGL (-Z forward). Phase 1 testing catches this before any Gemini integration.
2. **Gemini API latency**: 2-5s per call (5-10s total). Run in background thread; acceptable for one-shot paradigm.
3. **MoveIt planning failures**: May fail in cluttered scenes or near workspace boundary. Mitigate with retry logic and reasonable `planning_time`.
4. **Grasp orientation**: UR5e tool0 frame differs from WidowX. The quaternion for "gripper pointing down" needs calibration in Phase 5.
5. **Height calibration**: `table_z`, `grasp_height`, `transit_height` all depend on the Isaac Sim scene setup and will need tuning.

---

## Dependencies

- `google-genai` Python package (install in `.venv` or system-wide)
- `moveit_commander` (comes with `ros-jazzy-moveit` already installed)
- `tf2_ros` (already available in ROS2 Jazzy)
- No new ROS packages need building
