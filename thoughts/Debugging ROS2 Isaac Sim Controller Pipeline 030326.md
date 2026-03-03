# Debugging ROS2 Isaac Sim Controller Pipeline

**Date:** March 3, 2026
**Session Start:** ~03:49 UTC

---

## Executive Summary

Resolved a multi-layered ROS2 + Isaac Sim integration failure that prevented MoveIt from commanding the UR5e arm. Root causes spanned four distinct layers: a missing hardware plugin, stale background processes, a broken OmniGraph timestamp pipeline, and a sim-time/wall-time mismatch in `move_group`. The arm is now fully teleoperable from RViz via MoveIt with Isaac Sim as the physics backend.

---

## Problem Chain

### 1. Duplicate `ros2_control` Hardware Blocks
`ur5e.urdf.xacro` was including `ur5e_isaac_sim.urdf.xacro` (which already contained a `ros2_control` block using `topic_based_ros2_control/TopicBasedSystem`) while also injecting a second block via `ur5e.ros2_control.xacro` (`mock_components/GenericSystem`). Two hardware plugins claimed the same 6 joints. The `topic_based_ros2_control` plugin was also not installed.

**Fix:** Removed the redundant `mock_components` block from `ur5e.urdf.xacro`. Built `topic_based_ros2_control` from source (no Jazzy apt package available).

### 2. Stale Controller Manager Processes
After kills and restarts, two `/controller_manager` nodes were discovered running simultaneously — one from a previous session that was never cleaned up.

**Fix:** `killall -9 ros2_control_node move_group robot_state_publisher rviz2` before each relaunch.

### 3. Isaac Sim OmniGraph Timestamp Always Zero
Isaac Sim was publishing `/clock` and `/joint_states` at 60 Hz but with `stamp: sec=0, nanosec=0`. The root `ActionGraph` had its `inputs:timeStamp` connections wired to three nodes (`on_playback_tick`, `on_tick`, `isaac_read_simulation_time`) — none of which existed in the graph. The only real trigger (`on_physics_step`) had its time outputs explicitly deleted from the timestamp connection.

**Fix (iterative):**
- Added `isaac_read_simulation_time` (`isaacsim.core.nodes.IsaacReadSimulationTime`) to the root ActionGraph
- Cleaned up multi-input conflicts on `inputs:timeStamp` (OmniGraph throws `allowMultiInputs` warning and may bail on node execution when multiple real connections exist without the metadata)
- Kept `on_physics_step` as the sole `execIn` trigger; `isaac_read_simulation_time` as the sole timestamp source

Controllers activated immediately once the sim clock was properly advancing.

### 4. `move_group` Using Wall Clock vs. Sim Clock
After controllers came up, MoveIt still rejected joint states with:
```
Requested time 1772540917.767918, but latest received state has time 276.550014
```
`generate_move_group_launch()` from `moveit_configs_utils` does not forward `use_sim_time`. All other nodes (robot_state_publisher, controller_manager, rviz2) had it set explicitly, but `move_group` was defaulting to wall time (~1.77 billion seconds) while joint states were stamped with sim time (~276 seconds).

**Fix:** Replaced `generate_move_group_launch(moveit_config)` with a manual `Node(package="moveit_ros_move_group", ...)` call with `{"use_sim_time": True}` in parameters.

---

## Current State (End of Session)

- Hardware: `ur5e_isaac` (`topic_based_ros2_control/TopicBasedSystem`) — **active**
- Controllers: `joint_state_broadcaster`, `ur_manipulator_controller` — **active**
- Clock: Isaac Sim publishing real sim time via `isaac_read_simulation_time`
- MoveIt: Planning succeeds; open issue with OMPL failing after first execution (likely SRDF collision pair gaps or Isaac Sim physics drift)

## Open Issues

- OMPL times out (5 s) on second+ planning attempt — suspected cause: incomplete `disable_collisions` entries in SRDF, or Isaac Sim joint positions drifting post-execution placing the arm near self-collision in the planning scene
- RViz crashes on launch due to snap/glibc conflict (`libpthread.so.0: undefined symbol: __libc_pthread_init`) — unrelated to controller stack, snap environment interference

## Key Lessons

- Always source ROS2 and workspace before launching; stale processes from previous sessions persist silently
- `generate_move_group_launch` does **not** pass `use_sim_time` — always define `move_group` as an explicit Node when using sim time
- `topic_based_ros2_control` is not in the Jazzy apt repos — must build from source (PickNik: `github.com/PickNikRobotics/topic_based_ros2_control`)
- OmniGraph `allowMultiInputs` is required metadata when connecting multiple upstream data nodes to a single input; without it, real connections (not just dangling ones) can cause the node to stop executing
- Isaac Sim must be in **Play** mode before launching the ROS2 controller stack — the controller manager RT loop is driven by `/clock`, which is only published during playback
