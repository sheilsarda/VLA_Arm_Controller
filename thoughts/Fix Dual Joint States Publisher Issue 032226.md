# Fix: Dual `/joint_states` Publisher Issue (#6)

## Context

Two ROS2 publishers compete on `/joint_states`, corrupting training data and creating runtime risk:

1. **Isaac Sim ROS2 Bridge** (USDA OmniGraph node) — publishes real joint positions but fires twice per cycle due to dual execution triggers (`on_playback_tick` + `on_physics_step`)
2. **`joint_state_broadcaster`** (ros2_control) — reads state from `TopicBasedSystem`, which subscribes to `/joint_states` itself, creating a **feedback loop**. It re-publishes whatever it last read (often near-zero startup values) back onto the same topic.

Result: ~50% of messages on `/joint_states` contain near-zero garbage. The previous fix attempt (removing the Isaac Sim publisher entirely) caused ALL positions to go to zero because `TopicBasedSystem` lost its only source of real joint states.

## Why `joint_state_broadcaster` is redundant here

In standard ros2_control, `joint_state_broadcaster` bridges the controller_manager's internal state interfaces to the `/joint_states` ROS2 topic. But in this project, Isaac Sim's `ros2_publish_joint_state` OmniGraph node already publishes ground-truth joint states directly onto `/joint_states`. The broadcaster just echoes what Isaac Sim already publishes — creating a feedback loop in the process.

Consumers of `/joint_states` (`vla_controller_node`, `robot_state_publisher`, `ros2 bag record`, `eval_recorder`) all work identically reading from Isaac Sim directly.

Note: if this project later moves to real hardware (where the hardware interface isn't topic-based), `joint_state_broadcaster` would need to be re-added as the canonical `/joint_states` source.

## Fix: Remove `joint_state_broadcaster` + Fix Dual Trigger

### Data flow after fix
```
Isaac Sim physics engine
  |
  v
ros2_publish_joint_state (OmniGraph)
  - Triggered ONLY by on_physics_step (~60Hz)
  - Publishes to: /joint_states (SOLE publisher)
  |
  v
/joint_states topic (single clean source)
  |
  +---> TopicBasedSystem (subscribes, populates state interfaces for controller_manager)
  |       |
  |       v
  |     controller_manager
  |       |
  |       +---> ur_manipulator_controller --> /joint_commands --> Isaac Sim
  |       +---> gripper_controller --> /joint_commands --> Isaac Sim
  |
  +---> vla_controller_node (reads current joint positions)
  +---> robot_state_publisher (computes TF transforms)
  +---> ros2 bag record (training data)
```

### Step 1: Fix USDA file — remove dual trigger

**File:** `Simulator_Assets/ur5e_from_urdf_script.usda`

Two changes in the `ros2_publish_joint_state` node (lines 642-679):

**A. Single execution trigger** (lines 653-656) — remove `on_playback_tick`:
```
# Before:
prepend uint inputs:execIn.connect = [
    </ActionGraph/on_playback_tick.outputs:tick>,
    </ActionGraph/on_physics_step.outputs:step>,
]

# After:
prepend uint inputs:execIn.connect = </ActionGraph/on_physics_step.outputs:step>
```

**B. Single timestamp source** (lines 670-673) — remove `on_playback_tick`:
```
# Before:
prepend double inputs:timeStamp.connect = [
    </ActionGraph/on_playback_tick.outputs:time>,
    </ActionGraph/isaac_read_simulation_time.outputs:simulationTime>,
]

# After:
prepend double inputs:timeStamp.connect = </ActionGraph/isaac_read_simulation_time.outputs:simulationTime>
```

### Step 2: Remove `joint_state_broadcaster`

**File:** `src/ur5e_isaac_moveit_config/config/ros2_controllers.yaml`

Remove these lines:
```yaml
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

### Step 3: Rebuild

```bash
cd ~/Development/VLA_Arm_Controller
colcon build --packages-select ur5e_isaac_moveit_config
source install/setup.bash
```

## What does NOT change

- **`urdf/ur5e_isaac_sim.urdf.xacro`** — `TopicBasedSystem` keeps subscribing to `/joint_states` (now clean, single source)
- **`vla_params.yaml`** — `joint_state_topic: "/joint_states"` stays as-is
- **`vla_system.launch.py`** — no topic names referenced directly
- **`topic_based_system.cpp`** — no code change needed
- **Training recording** — `ros2 bag record /joint_states` continues to work, now clean
- **`analyze_training_episodes.py`** — existing 10ms dedup filter still works as a safety net

## Verification

1. Launch Isaac Sim + `vla_system.launch.py`
2. `ros2 topic info /joint_states` — confirm exactly 1 publisher (Isaac Sim bridge, NOT joint_state_broadcaster)
3. `ros2 topic echo /joint_states --once` — confirm real (non-zero) positions
4. `ros2 topic hz /joint_states` — confirm stable rate (~60Hz from Isaac Sim, no interleaved duplicates)
5. Record a short bag, run `analyze_training_episodes.py` — confirm no interleaved near-zero values
6. Run an eval with `dry_run:=true` — confirm `vla_controller_node` reads correct joint positions
