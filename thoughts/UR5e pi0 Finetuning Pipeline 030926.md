# UR5e pi0 Fine-Tuning Pipeline

**Date:** March 9, 2026

---

## Goal

Fine-tune a `pi0` policy on UR5e Isaac Sim demonstrations so the VLA acts correctly on our robot. The off-the-shelf `pi05_droid` model produces nonsensical motions because it was trained on Franka EEF-velocity actions in a different visual domain — fine-tuning on our own data is required.

---

## Why DROID Zero-Shot Fails

| Mismatch | DROID baseline | Our target |
|---|---|---|
| Robot | Franka Panda (7-DOF) | UR5e (6-DOF) |
| Action type | EEF velocity (8D) | Delta joint angles + gripper (7D) |
| Visual domain | Real-world lab scenes | Isaac Sim renderer |
| Action horizon | 16 | TBD (start with 16) |

---

## Pipeline Overview

```
Isaac Sim (scripted motions)
        │
        │  ros2 bag record
        ▼
ROS2 Bag files  (.db3 / .mcap)
  /camera/image_raw
  /camera_wrist/image_raw
  /joint_states
        │
        │  convert_ur5_bag_to_lerobot.py
        ▼
LeRobot Dataset (local or HuggingFace Hub)
  exterior_image_1_left  (320×240, uint8)
  wrist_image_left       (320×240, uint8)
  joint_position         (6,) float32
  gripper_position       (1,) float32
  actions                (7,) float32  [Δj0..j5 | gripper_abs]
        │
        │  compute_norm_stats.py + train.py
        ▼
Fine-tuned pi0_ur5 checkpoint
        │
        │  serve_policy.py --policy.config=pi0_ur5
        ▼
openpi inference server  →  vla_controller_node  →  Isaac Sim
```

---

## Files to Create / Modify

### New files

| File | Location | Purpose |
|---|---|---|
| `record_episode.sh` | `VLA_Arm_Controller/scripts/` | Wrapper around `ros2 bag record` with correct topics |
| `convert_ur5_bag_to_lerobot.py` | `openpi/examples/ur5/` | Reads bags, syncs at 10 Hz, writes LeRobot dataset |
| `ur5_policy.py` | `openpi/src/openpi/policies/` | UR5-specific Inputs/Outputs transforms |

### Modified files

| File | Change |
|---|---|
| `openpi/src/openpi/training/config.py` | Add `pi0_ur5` `TrainConfig` + import `ur5_policy` |

---

## Step-by-Step Plan

### Step 1 — Record Episodes (`record_episode.sh`)

Script wraps `ros2 bag record` for the three relevant topics:

```bash
ros2 bag record \
  /camera/image_raw \
  /camera_wrist/image_raw \
  /joint_states \
  -o "$OUTPUT_DIR/episode_$EPISODE_ID"
```

- Run the scripted MoveIt pick-and-place controller simultaneously.
- Start/stop the bag manually (Ctrl+C) or via a trigger topic.
- **Target:** 50–100 episodes, 15–30 s each, single task ("pick up the block").

### Step 2 — Convert Bags to LeRobot (`convert_ur5_bag_to_lerobot.py`)

Uses the `rosbags` pip package (no ROS2 install required at conversion time).

Key logic:
1. Open each bag with `rosbags.rosbag2.Reader`.
2. Buffer all messages by topic → `(timestamp_ns, msg)` lists.
3. Subsample to **10 Hz**: for each target frame time, pick nearest message per topic.
4. Compute actions:
   - `delta_joints[t] = joint_pos[t+1] - joint_pos[t]`  (6D)
   - `gripper_abs[t]  = gripper_pos[t+1]`                (1D, absolute target)
   - Drop the last frame (no next state to diff against).
5. Resize images to **320×240** (training pipeline handles final resize+pad to 224×224).
6. Write to `LeRobotDataset` via `lerobot` API (same pattern as DROID converter).

Dataset features schema:

```python
{
  "exterior_image_1_left": {"dtype": "image", "shape": (240, 320, 3)},
  "wrist_image_left":      {"dtype": "image", "shape": (240, 320, 3)},
  "joint_position":        {"dtype": "float32", "shape": (6,)},
  "gripper_position":      {"dtype": "float32", "shape": (1,)},
  "actions":               {"dtype": "float32", "shape": (7,)},
}
```

CLI usage:
```bash
uv run examples/ur5/convert_ur5_bag_to_lerobot.py \
    --bags-dir /path/to/bags \
    --repo-id your_hf_username/ur5_isaac_sim_v1 \
    --task "pick up the block" \
    [--output-fps 10] \
    [--push-to-hub]
```

### Step 3 — `ur5_policy.py`

Mirrors `droid_policy.py` with two differences:

- **State:** pad `joint_position` (6,) → (7,) with a zero, concat `gripper_position` (1,) → **8D state** matching the pretrained weight's state embedding.
- **Actions:** slice `actions[:, :7]` (not `[:, :8]`).

```python
class UR5Inputs(transforms.DataTransformFn):
    model_type: _model.ModelType
    def __call__(self, data):
        joints = np.asarray(data["observation/joint_position"])   # (6,)
        padded = np.concatenate([joints, np.zeros(1)])            # (7,)
        gripper = np.asarray(data["observation/gripper_position"]) # (1,)
        state = np.concatenate([padded, gripper])                  # (8,)
        # same image handling as DroidInputs …

class UR5Outputs(transforms.DataTransformFn):
    def __call__(self, data):
        return {"actions": np.asarray(data["actions"][:, :7])}
```

Also add `make_ur5_example()` for smoke-testing the policy config.

### Step 4 — `pi0_ur5` TrainConfig

Add to `_CONFIGS` list in `config.py` (before the debug configs):

```python
TrainConfig(
    name="pi0_ur5",
    model=pi0_config.Pi0Config(
        action_dim=7,
        action_horizon=16,
    ),
    data=LeRobotUR5DataConfig(
        repo_id="your_hf_username/ur5_isaac_sim_v1",
        assets=AssetsConfig(asset_id="ur5e"),
    ),
    weight_loader=weight_loaders.CheckpointWeightLoader(
        "gs://openpi-assets/checkpoints/pi0_base/params"
    ),
    num_train_steps=20_000,
    batch_size=32,
),
```

`LeRobotUR5DataConfig` (also new, in `config.py`) mirrors `LeRobotDROIDDataConfig` but uses `UR5Inputs`/`UR5Outputs` and the UR5 repack keys.

### Step 5 — Compute Norm Stats

```bash
cd /home/sheil/Development/openpi
uv run scripts/compute_norm_stats.py --config-name pi0_ur5
```

Writes stats to `./assets/ur5e/`.

### Step 6 — Fine-Tune

```bash
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 uv run scripts/train.py pi0_ur5 \
    --exp-name=ur5_isaac_pick_v1
```

- Checkpoint interval: 5 000 steps.
- Evaluate at 10 k and 20 k steps by serving the checkpoint and running a live episode.

### Step 7 — Serve Fine-Tuned Model

```bash
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_ur5 \
    --policy.dir=checkpoints/pi0_ur5/ur5_isaac_pick_v1/20000
```

Update `vla_params.yaml`: set `action_mode: delta` (instead of `velocity`).

---

## Dependencies

```bash
# Converter (outside ROS2 env)
pip install rosbags lerobot tyro opencv-python tqdm

# openpi server (existing uv env)
# No new deps — ur5_policy.py uses only existing openpi internals
```

---

## Open Questions / Risks

| # | Question | Impact |
|---|---|---|
| 1 | What resolution does Isaac Sim publish images at? | Affects storage size per episode; converter resizes to 320×240 anyway |
| 2 | Is 50 episodes enough for pi0 fine-tune? | May need 100–200; start small and evaluate |
| 3 | Gripper joint name in Isaac /joint_states? | Must match `gripper_joint` in `vla_params.yaml` and converter `--gripper-joint` arg |
| 4 | Does Isaac Sim bag record at stable rate? | May need to drop frames with large time gaps |
| 5 | Action horizon: 16 or shorter? | Shorter = more re-inference, more responsive; start with 16 |

---

## Suggested Implementation Order

1. `record_episode.sh` — collect first 10 test episodes
2. `convert_ur5_bag_to_lerobot.py` — validate dataset shape/content
3. `ur5_policy.py` + `LeRobotUR5DataConfig` + `pi0_ur5` TrainConfig
4. `compute_norm_stats.py`
5. Fine-tune at 10 k steps, evaluate live
6. Iterate: more data or more steps as needed
