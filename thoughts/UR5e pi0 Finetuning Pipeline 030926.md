# UR5e pi0 Fine-Tuning Pipeline

**Date:** March 9, 2026

---

## Goal

Validate the full fine-tuning pipeline end-to-end using a simple vertical arm motion task ("move the arm up and down"). No object manipulation — this is a pipeline proof-of-concept. Once the loop works (record → convert → train → serve → control), it can be repeated for any real task.

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
Isaac Sim (human teleop via MoveIt)
        │
        │  ros2 bag record  (bashrc alias: record_episode <id>)
        ▼
ROS2 Bag files  (.db3 / .mcap)  →  training_data/episode_<id>/
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
        │  LERO  (episode curation — delete bad demos, visual inspection)
        ▼
Curated LeRobot Dataset
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
| `convert_ur5_bag_to_lerobot.py` | `openpi/examples/ur5/` | Reads bags, syncs at 10 Hz, writes LeRobot dataset |
| `ur5_policy.py` | `openpi/src/openpi/policies/` | UR5-specific Inputs/Outputs transforms |

### Modified files

| File | Change |
|---|---|
| `openpi/src/openpi/training/config.py` | Add `pi0_ur5` `TrainConfig` + import `ur5_policy` |  

---

## Step-by-Step Plan

### Step 1 — Record Episodes

No dedicated script. Use two terminals:

- **Terminal 1:** run the MoveIt teleop controller and move the arm up and down vertically
- **Terminal 2:** start/stop the bag around each attempt

```bash
record_episode 001
```

(`record_episode` is a `~/.bashrc` function — see README.)

- **Target:** 20–30 episodes, 5–10 s each (one or two full up-down cycles).
- Bags are written to `VLA_Arm_Controller/training_data/` (gitignored).
- Only record clean attempts; discard anything that looks wrong before stopping the bag.

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
    --bags-dir ~/Development/VLA_Arm_Controller/training_data \
    --repo-id sheilsarda/ur5_isaac_sim_v1 \
    --task "move the arm up and down" \
    [--output-fps 10] \
    [--push-to-hub]
```

### Step 2b — Curate Dataset with LERO

Install: `pip install lero`

Use LERO's interactive GUI to visually inspect episodes and delete any bad ones before training.

```bash
lero gui --dataset sheilsarda/ur5_isaac_sim_v1
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

> **Status: DONE.** Config lives in `config.py`. Model was changed from full pi0 to **pi0-FAST LoRA** due to VRAM constraints (12GB RTX 4000 Ada — full pi0 needs ~48GB).

```python
TrainConfig(
    name="pi0_ur5",
    model=pi0_fast.Pi0FASTConfig(
        action_dim=8,          # 6 joints + 1 pad + 1 gripper
        action_horizon=10,
        max_token_len=180,
        paligemma_variant="gemma_2b_lora",
    ),
    data=LeRobotUR5DataConfig(
        repo_id="sheilsarda/ur5_isaac_sim_v1",
        base_config=DataConfig(prompt_from_task=True),
    ),
    weight_loader=weight_loaders.CheckpointWeightLoader(
        "gs://openpi-assets/checkpoints/pi0_fast_base/params"
    ),
    num_train_steps=5_000,
    freeze_filter=pi0_fast.Pi0FASTConfig(...).get_freeze_filter(),
    ema_decay=None,
    batch_size=2,
    num_workers=0,
),
```

`LeRobotUR5DataConfig` (in `config.py`) uses `Ur5Inputs`/`Ur5Outputs` from `src/openpi/policies/ur5_policy.py`. `Ur5Outputs` slices output back to `[:, :7]` (dropping the padding dim).

### Step 5 — Compute Norm Stats

> **Must re-run any time the model config or dataset changes.**

```bash
cd /home/sheil/Development/openpi
uv run scripts/compute_norm_stats.py --config-name=pi0_ur5
```

Writes stats to `./assets/pi0_ur5/sheilsarda/ur5_isaac_sim_v1/`.

### Step 6 — Fine-Tune

```bash
XLA_PYTHON_CLIENT_MEM_FRACTION=0.85 uv run scripts/train.py pi0_ur5 \
    --exp-name=ur5_lift_v1 --overwrite
```

- `XLA_PYTHON_CLIENT_MEM_FRACTION=0.85` — required to avoid OOM on 12GB VRAM
- Checkpoint interval: 1,000 steps (default)
- Checkpoints saved to `checkpoints/pi0_ur5/ur5_lift_v1/`
- Evaluate at 2–5k steps — the task is simple enough that it should work quickly if the pipeline is correct

### Step 7 — Serve Fine-Tuned Model

```bash
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
    --policy.config=pi0_ur5 \
    --policy.dir=checkpoints/pi0_ur5/ur5_lift_v1/5000
```

Update `vla_params.yaml`: set `action_mode: delta` (instead of `velocity`).

> **Next:** Write the UR5e inference client (`vla_controller_node`) that connects to the openpi server and sends observations / receives actions.

---

## Dependencies

```bash
# Converter + curation (outside ROS2 env)
pip install rosbags lerobot lero tyro opencv-python tqdm

# openpi server (existing uv env)
# No new deps — ur5_policy.py uses only existing openpi internals
```

---

## Open Questions / Risks

| # | Question | Impact |
|---|---|---|
| 1 | What resolution does Isaac Sim publish images at? | Affects storage size per episode; converter resizes to 320×240 anyway |
| 2 | Gripper joint name in Isaac /joint_states? | Must match `gripper_joint` in `vla_params.yaml` and converter `--gripper-joint` arg |
| 3 | Does Isaac Sim bag record at stable rate? | May need to drop frames with large time gaps |
| 4 | Action horizon: 16 or shorter? | Shorter = more re-inference, more responsive; start with 16 |

---

## Suggested Implementation Order

1. Collect 20–30 episodes with `record_episode <id>`
2. `convert_ur5_bag_to_lerobot.py` — validate dataset shape/content
3. Curate with LERO, delete any bad episodes
4. `ur5_policy.py` + `LeRobotUR5DataConfig` + `pi0_ur5` TrainConfig
5. `compute_norm_stats.py`
6. Fine-tune, evaluate live at 2–5k steps
