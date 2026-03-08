# VLA Integration Plan for UR5e / Isaac Sim

**Date:** March 8, 2026

---

## Goal

Add an optional VLA (Vision-Language-Action) control mode to the existing ROS2 + MoveIt + Isaac Sim pipeline. Instead of a human driving the arm via RViz, a VLA model takes camera images and a language instruction as input and outputs end-effector trajectories that get executed on the UR5e.

Two models to support:
- **X-VLA** (`lerobot/xvla-widowx` on HuggingFace, via `lerobot` library)
- **VLAC** (HuggingFace, interface TBD — awaiting model details)

Reference workflow: `Docs/Xvla_widowx_vis_traj.ipynb` (X-VLA on WidowX 250s in MuJoCo)

---

## Architecture

```
Isaac Sim
  ├── /camera/image_raw        (sensor_msgs/Image)
  ├── /camera_side/image_raw   (optional second camera)
  └── /joint_states            (via joint_state_broadcaster)
              │
              ▼
  [vla_controller_node]
  ├── Loads selected model backend (X-VLA or VLAC)
  ├── Subscribes to camera topics + reads EE state via FK
  ├── Accepts language instruction as ROS param / topic
  ├── Runs inference → action chunk (K × 7D EE waypoints)
  └── Converts EE poses → joint positions via MoveIt compute_ik
              │
              ▼
  [JointTrajectoryController] → Isaac Sim joints
```

MoveGroup still runs in VLA mode (needed for the IK service). RViz optional.

---

## Work Phases

### Phase 1 — New `vla_controller` ROS2 Package

Create `src/vla_controller/` as a standalone ROS2 Python package.

**Files:**
```
src/vla_controller/
├── package.xml
├── CMakeLists.txt (or setup.py)
├── vla_controller/
│   ├── __init__.py
│   ├── vla_controller_node.py      # Main ROS2 node
│   ├── ik_bridge.py                # MoveIt compute_ik service wrapper
│   └── model_backends/
│       ├── __init__.py
│       ├── base.py                 # Abstract base: load(), infer(obs) → action_chunk
│       ├── xvla_backend.py         # Wraps lerobot XVLAPolicy
│       └── vlac_backend.py         # Wraps VLAC (TBD)
└── config/
    └── vla_params.yaml             # Camera topics, workspace transform, chunk config
```

**`base.py` interface:**
```python
class VLABackend(ABC):
    @abstractmethod
    def load(self, model_path: str, device: str): ...

    @abstractmethod
    def infer(self, observation: dict) -> np.ndarray:
        # observation keys: images (list), ee_state, language_tokens
        # returns: (K, 7) array of EE waypoints [x,y,z,rx,ry,rz,gripper]
        ...
```

### Phase 2 — Camera / State Bridge

Isaac Sim must publish camera images as ROS topics. Check if the current USD scene has cameras configured with OmniGraph ROS2 publishers. If not, add them.

Needed topics (configurable in `vla_params.yaml`):
- Primary: `/camera/image_raw`
- Side (optional): `/camera_side/image_raw`

EE state: computed from `/joint_states` using FK (via `tf2` or MoveIt FK service) — no new Isaac configuration needed.

### Phase 3 — X-VLA Backend

Based on the notebook workflow:

```python
# observation dict for XVLAPolicy.select_action()
observation = {
    'observation.images.image':            img_tensor,      # [1, C, H, W]
    'observation.images.image2':           img2_tensor,     # [1, C, H, W] (optional)
    'observation.state':                   ee_state_tensor, # [1, 8] (x,y,z,r,p,y,pad,gripper)
    'observation.language.tokens':         language_tokens,
    'observation.language.attention_mask': attn_mask,
}
actions = policy.select_action(observation)
# actions: (chunk_size, 7) EE waypoints
```

Action mode is `ee6d` — absolute end-effector poses in the robot's base frame.

### Phase 4 — VLAC Backend

Interface TBD. Awaiting:
- HuggingFace model ID
- Action space (joint-space vs. EE-space, absolute vs. delta)
- Required observation keys
- Library dependency (lerobot, custom, transformers)

Will implement to the same `VLABackend` interface once known.

### Phase 5 — IK Bridge (`ik_bridge.py`)

Convert EE pose waypoints → joint positions using MoveIt's `compute_ik` service:

```
/compute_ik  (moveit_msgs/srv/GetPositionIK)
```

Steps per waypoint:
1. Build `geometry_msgs/PoseStamped` from [x,y,z,rx,ry,rz]
2. Call `/compute_ik` with current joint state as seed
3. Extract joint positions from response
4. Append to `trajectory_msgs/JointTrajectory`

Send complete trajectory to `/ur_manipulator_controller/follow_joint_trajectory` action.

### Phase 6 — Execution Loop

```
Node startup:
  - load model backend
  - tokenize language instruction (cached)
  - subscribe to camera + joint_state topics

Main loop (rate: configurable, e.g. 2 Hz for re-inference):
  1. Grab latest image(s)
  2. Compute current EE state via FK
  3. Run VLA inference → action chunk (K waypoints)
  4. IK-solve each waypoint → joint trajectory
  5. Send trajectory to controller
  6. Wait for execution window (chunk_execution_time)
  7. Repeat
```

Key parameters (in `vla_params.yaml`):
- `chunk_size`: waypoints to execute before re-inferring (tune latency vs. reactivity)
- `waypoint_dt`: time between waypoints (seconds)
- `workspace_offset`: [x, y, z] offset to align model's frame to UR5e base frame
- `workspace_scale`: scalar to rescale model outputs to UR5e workspace

### Phase 7 — Launch Integration

New `vla.launch.py` (or add args to `controller_v1.launch.py`):

```python
use_vla              = LaunchArgument("use_vla", default="false")
vla_model            = LaunchArgument("vla_model", default="xvla")  # "xvla" | "vlac"
task_instruction     = LaunchArgument("task", default="pick up the block")
vla_model_path       = LaunchArgument("vla_model_path", default="")  # local path or HF ID
```

When `use_vla=true`:
- Launch `vla_controller_node` with above params
- Still launch MoveGroup (IK service dependency)
- RViz optional (can be disabled)

---

## Known Unknowns / Open Questions

| # | Question | Needed For |
|---|----------|------------|
| 1 | VLAC HuggingFace model ID + action space | Phase 4 |
| 2 | X-VLA checkpoint to use (widowx or other) | Phase 3 |
| 3 | Camera topics Isaac Sim currently publishes (if any) | Phase 2 |
| 4 | Gripper present on UR5e in Isaac scene? | Gripper action handling |
| 5 | GPU available on host machine for inference? | Hardware requirements |

---

## Coordinate Frame Mismatch Note

X-VLA is trained on WidowX 250s data (BridgeData V2). The output EE poses are in the WidowX base frame and workspace. The UR5e has a different workspace size, base height, and default orientation. Initial integration will use a configurable `workspace_offset` and `workspace_scale` in `vla_params.yaml` and tune empirically. Eventually, fine-tuning on UR5e data or using a model trained on UR5e/similar would resolve this properly.

---

## Dependencies to Add

```
# Python
lerobot[xvla]       # X-VLA
torch, torchvision
transformers
Pillow

# ROS2 (apt)
ros-jazzy-moveit-msgs   # for compute_ik service types
ros-jazzy-cv-bridge     # sensor_msgs/Image ↔ numpy
ros-jazzy-vision-opencv
```

---

## Suggested Implementation Order

1. `base.py` + `xvla_backend.py` (model-agnostic scaffolding + first backend)
2. `ik_bridge.py` (test standalone against running MoveGroup)
3. `vla_controller_node.py` (wire together with dummy/mock backend first)
4. `vla.launch.py`
5. `vlac_backend.py` (once model info available)
6. Workspace calibration / tuning
