# VLA Integration Plan for UR5e / Isaac Sim

**Date:** March 8, 2026
**Last Updated:** March 9, 2026

---

## Goal

Add an optional VLA (Vision-Language-Action) control mode to the existing ROS2 + Isaac Sim pipeline. A language instruction and camera images are sent to a **pi0/pi0.5 inference server** (openpi), which returns delta joint commands executed directly on the UR5e — no IK solving required.

Model: **pi0 / pi0.5** via [openpi](https://github.com/Physical-Intelligence/openpi)
Reference: `examples/ur5/` in the openpi repo

## Reality Check (March 9, 2026)

- `pi0_ur5` is **not** registered in upstream `src/openpi/training/config.py`, so `--policy.config=pi0_ur5` fails today.
- `pi0_base` is a checkpoint path, not a runnable config name for `serve_policy.py`.
- `openpi/examples/ur5/README.md` is a template/guide; it does not ship a ready UR5 policy config or a ready `convert_ur5_data_to_lerobot.py` script.

## Bridge Simplification (March 9, 2026)

- The ROS2 bridge now uses a **single observation schema**: DROID-compatible keys.
- We intentionally removed runtime schema switching (`droid|ur5|direct`) to reduce bring-up/debug complexity.
- Once a local `pi0_ur5` config exists in our openpi fork, we can reintroduce a UR5-native schema behind a clean feature flag.

## Insights from `sim-evals` (`droid_jointpos.py`)

From `arhanjain/sim-evals` ([`src/sim_evals/inference/droid_jointpos.py`](https://github.com/arhanjain/sim-evals/blob/main/src/sim_evals/inference/droid_jointpos.py)), we should adopt:
- **Chunk caching + open-loop horizon**: only call server when current chunk is exhausted.
- **Episode reset hook**: on episode reset, clear cached chunk/index (`reset()` behavior).
- **Image preprocessing parity**: use `resize_with_pad(..., 224, 224)` instead of plain resize to avoid aspect distortion.
- **Gripper post-processing**: threshold final gripper dim at `0.5` to robustly map to binary open/close commands.
- **Torch-to-NumPy extraction path**: simulator tensors are converted with `.detach().cpu().numpy()` before request packing.
- **Debug visualization path**: concatenate base+wrist resized images for quick runtime sanity checks.

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
  ├── Resizes images to uint8 with pad (224×224, RGB)
  ├── Reads joint positions (6) + gripper (1) from /joint_states
  ├── Caches action chunk + cursor (open_loop_horizon)
  └── Sends observation dict via WebSocket (msgpack_numpy) only when chunk refresh is needed
              │
              │    ws://localhost:8000
              ▼
  [openpi inference server]   (separate process, GPU machine)
  ├── Runs pi0 / pi0.5 policy
  ├── Applies normalization for selected config (DROID now, UR5e after custom config)
  └── Returns action chunk: (horizon, action_dim) float32
              │
              ▼
  [vla_controller_node continues]
  ├── Integrates deltas: new_joints[i] = new_joints[i-1] + action[i, :6]
  ├── Clips to joint limits
  ├── Maps action[i, 6] → gripper (threshold 0.5 by default)
  └── Publishes JointTrajectory
              │
              ▼
  [JointTrajectoryController] → Isaac Sim joints
```

**MoveGroup is not required in VLA mode** — pi0 outputs joint-space delta commands directly. No IK solving needed.

---

## Key Observation / Action Format

### Observation sent to server (single schema)

| Key | Shape | dtype | Source |
|-----|-------|-------|--------|
| `observation/exterior_image_1_left` | (224, 224, 3) | uint8 | `/camera/image_raw` |
| `observation/wrist_image_left` | (224, 224, 3) | uint8 | `/camera_wrist/image_raw` |
| `observation/joint_position` | (7,) | float32 | joints[0:6] + one padded zero |
| `observation/gripper_position` | (1,) | float32 | gripper from `/joint_states` |
| `prompt` | str | — | task instruction ROS param |

### Action returned by server

```
action_chunk: (horizon, action_dim) float32
  UR5 target config: action_dim = 7
  DROID bootstrap config: action_dim = 8
  UR5 target semantics:
    dims 0–5: delta joint angles   (radians, relative to current state)
    dim  6:   gripper command      (absolute: 0.0 = open, 1.0 = closed)
  DROID bootstrap semantics differ (joint-velocity-style 8D action); use only for connectivity smoke tests.
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

**Start server now (works out of the box):**
```bash
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py --env=DROID

# equivalent explicit form:
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi05_droid \
    --policy.dir=gs://openpi-assets/checkpoints/pi05_droid
```

**Start server (UR5 custom config, after local openpi changes):**
```bash
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_ur5 \
    --policy.dir=checkpoints/pi0_ur5/my_experiment/30000
```

The first command is for immediate connectivity tests. The second command only works after we add a UR5 config to the openpi fork.

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

### Camera Viewpoint Decision (Initial)

Use a **right-oblique exterior camera** as the default base camera mapping to `observation/exterior_image_1_left`.

Rationale:
- `sim-evals` `droid_jointpos.py` feeds `right_image` into `observation/exterior_image_1_left`.
- This is closer to a side/oblique DROID-like viewpoint than a top-down camera.
- It usually preserves both object geometry and gripper approach direction better than overhead-only views.

Recommended first pose (world frame, tune in Isaac viewport):
- Base camera position: `(x=0.80, y=-0.55, z=0.75)`
- Look-at target: `(x=0.45, y=0.00, z=0.08)`
- FOV target: `60-75 deg` horizontal

Wrist camera mount reference:
- Parent: `tool0` (URDF comment: tool0 axes are `X+ left, Y+ up, Z+ front`)
- Start offset from `tool0`: `(x=0.00, y=0.02, z=0.07)` meters
- Point optical axis roughly along gripper forward direction (`tool0 +Z`) with slight downward pitch.

---

## Work Phases

### Phase 1 — openpi Server Setup and Smoke Test

1. Clone and install openpi (see Component 1 above).

2. Start server with built-in config (`--env=droid` or `pi05_droid`).

3. Smoke test with a minimal Python client (no ROS2):
   ```python
   from openpi_client import websocket_client_policy
   import numpy as np

   client = websocket_client_policy.WebsocketClientPolicy(host="localhost", port=8000)
   obs = {
       "observation/exterior_image_1_left": np.zeros((224, 224, 3), dtype=np.uint8),
       "observation/wrist_image_left":      np.zeros((224, 224, 3), dtype=np.uint8),
       "observation/joint_position":        np.zeros(7, dtype=np.float32),
       "observation/gripper_position":      np.zeros(1, dtype=np.float32),
       "prompt":            "pick up the block",
   }
   result = client.infer(obs)
   print(result["actions"].shape)   # expect (horizon, 8) for droid config
   ```

4. Treat Phase 1 as transport/protocol validation only. Do not treat DROID action semantics as UR5 control logic.

### Phase 2 — Isaac Sim Camera Setup

Verify or add the two cameras in the USD scene.

- Current status in this repo: top-level stage has no dedicated ROS image camera nodes yet (only viewport camera `/OmniverseKit_Persp`), so we should add both base+wrist cameras.
- Add `Camera` prims at the poses above and connect each to an OmniGraph `ROS2PublishImage` node.
- If cameras already exist in a different loaded layer at runtime, verify topic names and remap to the expected topics.

Validate with:
```bash
ros2 topic echo /camera/image_raw --once
ros2 topic echo /camera_wrist/image_raw --once
```

Camera acceptance checks (before training/fine-tuning):
- Base camera sees full manipulation zone and at least one gripper fingertip at nominal start pose.
- Wrist camera sees fingertips + near-object region without severe clipping.
- Across a short scripted motion, both streams stay informative (no persistent self-occlusion / no blank frames).

### Phase 3 — Image Bridge (`image_bridge.py`)

```python
from openpi_client import image_tools

class ImageBridge:
    def __init__(self):
        self.bridge = CvBridge()

    def convert(self, ros_image: Image) -> np.ndarray:
        cv_img = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        padded = image_tools.resize_with_pad(cv_img, 224, 224)
        return image_tools.convert_to_uint8(padded)   # (224, 224, 3)
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
  3. If `actions_from_chunk_completed == 0` or `>= open_loop_horizon`, pack observation:
       observation/exterior_image_1_left = image_bridge.convert(base_image)
       observation/wrist_image_left      = image_bridge.convert(wrist_image)
       observation/joint_position        = np.array([*joints, 0.0], np.float32)
       observation/gripper_position      = np.array([gripper], np.float32)
       prompt            = task_instruction
  4. If refresh needed: `client.infer(obs) -> pred_action_chunk`
  5. Consume next action (or short sub-chunk) from `pred_action_chunk`
  6. Apply post-processing: clip joints to limits, binarize gripper with threshold (default 0.5)
  7. Send trajectory to follow_joint_trajectory action server
  8. Increment `actions_from_chunk_completed`, repeat

Episode reset behavior:
  - Set `actions_from_chunk_completed = 0`
  - Set `pred_action_chunk = None`

Optional debugging:
  - Publish side-by-side (base + wrist) 224x224 view to a debug topic/image window to verify model inputs online.
```

**`vla_params.yaml`:**
```yaml
openpi_host: "localhost"
openpi_port: 8000
base_camera_topic:  "/camera/image_raw"
wrist_camera_topic: "/camera_wrist/image_raw"
task_instruction: "pick up the block"
open_loop_horizon: 5 # actions to execute before re-inferring
waypoint_dt: 0.1     # seconds between waypoints
gripper_threshold: 0.5
publish_debug_viz: true
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

### Phase 7 — Add Local UR5 Config to openpi

Create a small openpi fork patch so UR5 is a real runnable config.

Required additions (based on `openpi/examples/ur5/README.md` template):
- `src/openpi/policies/ur5_policy.py` with `UR5Inputs` and `UR5Outputs`
- New `TrainConfig` entry in `src/openpi/training/config.py`:
  - `name="pi0_ur5"`
  - `model=pi0_config.Pi0Config(action_horizon=<chosen>)`
  - `data=SimpleDataConfig(... assets=AssetsConfig(asset_id="ur5e"), data_transforms=UR5 transforms ...)`
  - `weight_loader=CheckpointWeightLoader("gs://openpi-assets/checkpoints/pi0_base/params")`

After this patch, run zero-shot UR5 server from the base checkpoint:
```bash
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_ur5 \
    --policy.dir=gs://openpi-assets/checkpoints/pi0_base
```

### Phase 8 — Fine-tune UR5 (if zero-shot is insufficient)

Collect UR5e demonstrations in Isaac Sim, then fine-tune.

Note: upstream openpi does not currently provide `examples/ur5/convert_ur5_data_to_lerobot.py`; we need to create this converter (e.g., by adapting `examples/droid/convert_droid_data_to_lerobot.py` or `examples/libero/convert_libero_data_to_lerobot.py`).

```bash
# Convert Isaac Sim recordings to LeRobot format (custom script to be added)
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

---

## Known Unknowns / Open Questions

| # | Question | Needed For |
|---|----------|------------|
| 1 | Does the USD scene already have cameras configured? (base + wrist) | Phase 2 |
| 2 | Is a gripper present and actuated in the Isaac scene? | Phase 4 (gripper dim) |
| 3 | GPU available on inference machine? (Ampere+ recommended) | Phase 1 latency |
| 4 | Will zero-shot pi0 (after custom UR5 config) generalize to Isaac Sim visuals? | Determines if Phase 8 is needed |
| 5 | openpi server on same machine as Isaac Sim, or remote? | Network config in vla_params.yaml |
| 6 | Is right-oblique base view (chosen default) sufficient, or should we keep a left-oblique fallback profile? | Data collection + inference consistency |

---

## Delta Joint Note

pi0 outputs **delta** joint commands for dims 0–5, not absolute EE poses. Implications:
- **No IK solving required** — major simplification vs. the original X-VLA approach
- Drift can accumulate over long executions; re-inference every `open_loop_horizon` steps mitigates this
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
7. Phase 7 — Add `pi0_ur5` config/transforms to openpi and run base-checkpoint zero-shot
8. Phase 8 — Fine-tune if zero-shot performance is insufficient
