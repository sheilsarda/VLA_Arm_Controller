# VLA Integration Plan for UR5e / Isaac Sim

**Date:** March 8, 2026
**Last Updated:** March 8, 2026

---

## Goal

Add an optional VLA (Vision-Language-Action) control mode to the existing ROS2 + Isaac Sim pipeline. A language instruction and camera images are sent to a **pi0/pi0.5 inference server** (openpi), which returns delta joint commands executed directly on the UR5e — no IK solving required.

Model: **pi0 / pi0.5** via [openpi](https://github.com/Physical-Intelligence/openpi)
Reference: `examples/ur5/` in the openpi repo

---

## Architecture

```
Isaac Sim
  ├── /camera/image_raw          (base/overhead — sensor_msgs/Image)
  ├── /camera_wrist/image_raw    (wrist camera  — sensor_msgs/Image)
  └── /joint_states              (via joint_state_broadcaster)
              │
              ▼
  [vla_controller_node]   (ROS2 Python — WebSocket client)
  ├── Resizes images to uint8 (224×224, RGB)
  ├── Reads joint positions (6) + gripper (1) from /joint_states
  └── Sends observation dict via WebSocket (msgpack_numpy)
              │
              │    ws://localhost:8000
              ▼
  [openpi inference server]   (separate process, GPU machine)
  ├── Runs pi0 / pi0.5 policy
  ├── Applies UR5e normalization (asset_id="ur5e")
  └── Returns action chunk: (horizon, 7) float32
              │
              ▼
  [vla_controller_node continues]
  ├── Integrates deltas: new_joints[i] = new_joints[i-1] + action[i, :6]
  ├── Clips to joint limits
  ├── Maps action[i, 6] → gripper (absolute: 0=open, 1=closed)
  └── Publishes JointTrajectory
              │
              ▼
  [JointTrajectoryController] → Isaac Sim joints
```

**MoveGroup is not required in VLA mode** — pi0 outputs joint-space delta commands directly. No IK solving needed.

---

## Key Observation / Action Format

### Observation sent to server

| Key | Shape | dtype | Source |
|-----|-------|-------|--------|
| `base_0_rgb` | (224, 224, 3) | uint8 | `/camera/image_raw` |
| `left_wrist_0_rgb` | (224, 224, 3) | uint8 | `/camera_wrist/image_raw` |
| `right_wrist_0_rgb` | (224, 224, 3) | uint8 | zeros (UR5e has one wrist cam) |
| `state` | (7,) | float32 | joints[0:6] + gripper[0:1] from `/joint_states` |
| `prompt` | str | — | task instruction ROS param |

### Action returned by server

```
action_chunk: (horizon, 7) float32
  dims 0–5: delta joint angles   (radians, relative to current state)
  dim  6:   gripper command      (absolute: 0.0 = open, 1.0 = closed)
```

---

## System Components

### Component 1 — openpi Inference Server

Standalone process. Can run on a remote GPU machine or localhost.

**Setup:**
```bash
cd /home/sheil/Development
git clone https://github.com/Physical-Intelligence/openpi
cd openpi
pip install uv && uv sync
pip install -e packages/openpi-client   # also install in ROS2 Python env
```

openpi lives at `/home/sheil/Development/openpi` — a sibling to `VLA_Arm_Controller`.

**Start server (base pi0 model, zero-shot):**
```bash
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_base \
    --policy.dir=gs://openpi-assets/checkpoints/pi0_base
```

**Start server (fine-tuned UR5e checkpoint):**
```bash
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_ur5 \
    --policy.dir=checkpoints/pi0_ur5/my_experiment/30000
```

Server binds to `0.0.0.0:8000`. Protocol: WebSocket + MessagePack+NumPy (`msgpack-numpy`).

### Component 2 — `vla_controller` ROS2 Package

```
src/vla_controller/
├── package.xml
├── setup.py
├── vla_controller/
│   ├── __init__.py
│   ├── vla_controller_node.py   # Main node — WebSocket client + ROS2 bridge
│   ├── image_bridge.py          # sensor_msgs/Image → uint8 numpy (224×224 RGB)
│   └── action_executor.py       # delta joints → JointTrajectory → controller
└── config/
    └── vla_params.yaml
```

No `model_backends/` — the model lives in the inference server, not the ROS2 node. The node is a pure bridge.

### Component 3 — Isaac Sim Cameras

Two cameras needed, each with an OmniGraph ROS2 Image publisher in the USD scene:

| Camera | Topic | Mount |
|--------|-------|-------|
| Base | `/camera/image_raw` | Fixed — overhead or angled workspace view |
| Wrist | `/camera_wrist/image_raw` | `tool0` link — moves with arm |

---

## Work Phases

### Phase 1 — openpi Server Setup and Smoke Test

1. Clone and install openpi (see Component 1 above).

2. Smoke test with a minimal Python client (no ROS2):
   ```python
   from openpi_client import websocket_client_policy
   import numpy as np

   client = websocket_client_policy.WebsocketClientPolicy(host="localhost", port=8000)
   obs = {
       "base_0_rgb":        np.zeros((224, 224, 3), dtype=np.uint8),
       "left_wrist_0_rgb":  np.zeros((224, 224, 3), dtype=np.uint8),
       "right_wrist_0_rgb": np.zeros((224, 224, 3), dtype=np.uint8),
       "state":             np.zeros(7, dtype=np.float32),
       "prompt":            "pick up the block",
   }
   result = client.infer(obs)
   print(result["actions"].shape)   # expect (horizon, 7)
   ```

### Phase 2 — Isaac Sim Camera Setup

Verify or add the two cameras in the USD scene.

- If cameras already exist as OmniGraph publishers: verify topic names and that images are being published at runtime.
- If not: add `Camera` prims at appropriate positions, connect each to an OmniGraph `ROS2PublishImage` node.

Validate with:
```bash
ros2 topic echo /camera/image_raw --once
ros2 topic echo /camera_wrist/image_raw --once
```

### Phase 3 — Image Bridge (`image_bridge.py`)

```python
class ImageBridge:
    def __init__(self):
        self.bridge = CvBridge()

    def convert(self, ros_image: Image) -> np.ndarray:
        cv_img = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        resized = cv2.resize(cv_img, (224, 224))
        return resized.astype(np.uint8)   # (224, 224, 3)
```

### Phase 4 — Action Executor (`action_executor.py`)

```python
def build_trajectory(current_joints, action_chunk, waypoint_dt, n_waypoints, joint_limits):
    """
    current_joints: (6,) float64 — current joint positions in radians
    action_chunk:   (horizon, 7) float32 — delta joints + abs gripper
    """
    points = []
    joints = current_joints.copy()
    for i in range(n_waypoints):
        joints = joints + action_chunk[i, :6].astype(float)
        joints = np.clip(joints, joint_limits[:, 0], joint_limits[:, 1])
        gripper = float(action_chunk[i, 6])

        point = JointTrajectoryPoint()
        point.positions = joints.tolist() + [gripper]
        point.time_from_start = Duration(seconds=waypoint_dt * (i + 1))
        points.append(point)
    return points
```

Send to `/ur_manipulator_controller/follow_joint_trajectory` via `FollowJointTrajectory` action client.

### Phase 5 — Main Node (`vla_controller_node.py`)

```
Node startup:
  - Load vla_params.yaml
  - Connect to openpi WebSocket (blocks with retries until server is up)
  - Subscribe to camera topics + /joint_states
  - Cache task_instruction from ROS param

Main loop (configurable rate, default 2 Hz):
  1. Grab latest base + wrist images
  2. Read current joint positions[0:6] + gripper[0:1] from /joint_states
  3. Pack observation:
       base_0_rgb        = image_bridge.convert(base_image)
       left_wrist_0_rgb  = image_bridge.convert(wrist_image)
       right_wrist_0_rgb = np.zeros((224,224,3), np.uint8)
       state             = np.array([*joints, gripper], np.float32)
       prompt            = task_instruction
  4. client.infer(obs) → action_chunk (horizon, 7)
  5. build_trajectory(current_joints, action_chunk[:chunk_size], ...)
  6. Send trajectory to follow_joint_trajectory action server
  7. Wait for execution window (chunk_size * waypoint_dt seconds)
  8. Repeat
```

**`vla_params.yaml`:**
```yaml
openpi_host: "localhost"
openpi_port: 8000
base_camera_topic:  "/camera/image_raw"
wrist_camera_topic: "/camera_wrist/image_raw"
task_instruction: "pick up the block"
chunk_size: 5        # waypoints to execute before re-inferring
waypoint_dt: 0.1     # seconds between waypoints
joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
gripper_joint: "finger_joint"   # set null if no gripper in scene
```

### Phase 6 — Launch Integration

New `vla.launch.py`:
```python
use_vla          = LaunchArgument("use_vla",       default="false")
task_instruction = LaunchArgument("task",           default="pick up the block")
openpi_host      = LaunchArgument("openpi_host",    default="localhost")
openpi_port      = LaunchArgument("openpi_port",    default="8000")
```

When `use_vla=true`:
- Launch `vla_controller_node`
- MoveGroup **not** launched
- RViz optional

### Phase 7 — Fine-tuning (if zero-shot is insufficient)

Collect UR5e demonstrations in Isaac Sim → convert to LeRobot format → fine-tune pi0.

```bash
# Convert Isaac Sim recordings to LeRobot format
cd /home/sheil/Development/openpi
python examples/ur5/convert_ur5_data_to_lerobot.py \
    --input-dir /path/to/episodes \
    --output-dir /path/to/lerobot_dataset

# Compute normalization stats
uv run scripts/compute_norm_stats.py --config-name pi0_ur5

# Fine-tune
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi0_ur5 \
    --exp-name=ur5_isaac_sim_v1

# Serve fine-tuned model
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_ur5 \
    --policy.dir=checkpoints/pi0_ur5/ur5_isaac_sim_v1/30000
```

See `/home/sheil/Development/openpi/examples/ur5/README.md` for full data format spec.

---

## Known Unknowns / Open Questions

| # | Question | Needed For |
|---|----------|------------|
| 1 | Does the USD scene already have cameras configured? (base + wrist) | Phase 2 |
| 2 | Is a gripper present and actuated in the Isaac scene? | Phase 4 (gripper dim) |
| 3 | GPU available on inference machine? (Ampere+ recommended) | Phase 1 latency |
| 4 | Will zero-shot pi0 generalize to Isaac Sim visuals? | Determines if Phase 7 is needed |
| 5 | openpi server on same machine as Isaac Sim, or remote? | Network config in vla_params.yaml |

---

## Delta Joint Note

pi0 outputs **delta** joint commands for dims 0–5, not absolute EE poses. Implications:
- **No IK solving required** — major simplification vs. the original X-VLA approach
- Drift can accumulate over long executions; re-inference every `chunk_size` steps mitigates this
- Joint limits must be enforced client-side before sending to the controller

---

## Dependencies

```bash
# openpi inference server (own venv, sibling directory)
cd /home/sheil/Development
git clone https://github.com/Physical-Intelligence/openpi
cd openpi && uv sync

# openpi client (install into ROS2 Python env too)
pip install -e /home/sheil/Development/openpi/packages/openpi-client

# ROS2 packages (apt)
ros-jazzy-cv-bridge
ros-jazzy-vision-opencv
ros-jazzy-control-msgs       # FollowJointTrajectory action type

# Python (ROS2 workspace)
msgpack-numpy
opencv-python
```

**No MoveIt, no lerobot, no torch dependency** in the ROS2 node — all heavy ML runs in the openpi server.

---

## Suggested Implementation Order

1. Phase 1 — Start openpi server, run smoke test client
2. Phase 2 — Verify/add cameras in Isaac Sim USD scene
3. Phase 3 — `image_bridge.py` — validate with saved test frames
4. Phase 4 — `action_executor.py` — test standalone against running controller
5. Phase 5 — `vla_controller_node.py` — wire together, test with mock server first
6. Phase 6 — `vla.launch.py`
7. Phase 7 — Fine-tune if zero-shot performance is insufficient
