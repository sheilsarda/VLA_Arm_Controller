# FAST Token Decoding Failure — Root Cause & Fix Plan

**Date:** March 22, 2026
**Status:** Active investigation
**Checkpoint:** `sheilsarda/pi0_ur5_fast_v1` (10k steps)
**W&B Run:** `ur5_fast_v1` at `wandb.ai/sheilsarda/openpi/runs/dtoe5fhc`

---

## Symptom

When running inference with the pi0-FAST UR5 model, 100% of action decodes fail with reshape errors. The arm receives all-zero action chunks and does not move.

### Raw Error Logs (from OpenPI server terminal)

```
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
INFO:openpi.serving.websocket_policy_server:Connection from ('127.0.0.1', 48414) closed
INFO:websockets.server:connection open
INFO:openpi.serving.websocket_policy_server:Connection from ('127.0.0.1', 56310) opened
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
Error decoding tokens: cannot reshape array of size 90 into shape (8)
Tokens: [1008, 289, 301, 341, 439, 439, 1345, 864]
INFO:openpi.serving.websocket_policy_server:Connection from ('127.0.0.1', 56310) closed
INFO:websockets.server:connection open
INFO:openpi.serving.websocket_policy_server:Connection from ('127.0.0.1', 54812) opened
Error decoding tokens: cannot reshape array of size 76 into shape (8)
Tokens: [300, 741, 293, 341, 439, 439, 273, 515, 530, 515, 278, 360, 290, 491, 428, 256]
Error decoding tokens: cannot reshape array of size 76 into shape (8)
Tokens: [300, 741, 293, 341, 439, 439, 273, 515, 530, 515, 278, 360, 290, 454, 428, 256]
Error decoding tokens: cannot reshape array of size 83 into shape (8)
Tokens: [300, 741, 293, 341, 439, 439, 822, 355, 278, 700, 881, 407, 856, 286, 856, 507, 258, 589, 273, 327, 593, 274, 324, 257]
Error decoding tokens: Decoded DCT coefficients have shape (11, 8), expected (10, 8)
Tokens: [300, 738, 293, 341, 439, 439, 822, 276, 807, 301, 287, 594, 1194, 273, 327, 534, 278, 258, 1776, 394, 257]
Error decoding tokens: cannot reshape array of size 75 into shape (8)
Tokens: [300, 741, 293, 341, 439, 439, 488, 301, 1260, 290, 327, 1194, 273, 327, 534, 278, 483, 395, 428, 256]
Error decoding tokens: cannot reshape array of size 87 into shape (8)
Tokens: [1008, 289, 293, 341, 439, 439, 576, 276, 733, 301, 834, 273, 394, 593, 274, 382, 407, 273, 327, 875, 293, 609, 268, 104, 974, 1021, 257]
Error decoding tokens: cannot reshape array of size 74 into shape (8)
Tokens: [300, 738, 293, 341, 439, 439, 576, 278, 360, 279, 360, 288, 1246, 324, 962, 360, 288]
Error decoding tokens: Decoded DCT coefficients have shape (12, 8), expected (10, 8)
Tokens: [300, 738, 293, 341, 439, 439, 266, 700, 276, 742, 290, 1892, 288, 291, 258, 589, 273, 327, 407, 273, 327, 268, 247, 1150, 919, 1931, 430, 289, 257]
Error decoding tokens: cannot reshape array of size 71 into shape (8)
Tokens: [300, 741, 293, 341, 439, 439, 274, 515, 530, 279, 1490, 340, 259, 589, 273, 394, 407, 273, 327, 428]
Error decoding tokens: cannot reshape array of size 67 into shape (8)
Tokens: [1008, 289, 293, 341, 412, 439, 413, 530, 491, 279, 360, 282, 292, 394, 428]
```

### Key observations from the logs

| Decoded DCT coeff count | Expected (`action_horizon * action_dim = 10 * 8`) | Off by |
|---|---|---|
| 90 | 80 | +10 (not divisible by 8) |
| 76 | 80 | -4 (not divisible by 8) |
| 83 | 80 | +3 (not divisible by 8) |
| 88 → shape (11,8) | 80 → shape (10,8) | +8 (divisible, but wrong horizon) |
| 75 | 80 | -5 (not divisible by 8) |
| 87 | 80 | +7 (not divisible by 8) |
| 74 | 80 | -6 (not divisible by 8) |
| 96 → shape (12,8) | 80 → shape (10,8) | +16 (divisible, but wrong horizon) |
| 71 | 80 | -9 (not divisible by 8) |
| 67 | 80 | -13 (not divisible by 8) |

The model produces variable-length BPE sequences near but never exactly 80 DCT coefficients.

---

## Error Location — Full Code Path

### 1. Server serve command

```bash
# Terminal 1 (openpi repo)
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi0_ur5 \
  --policy.dir=checkpoints/pi0_ur5/ur5_fast_v1/9999
```

### 2. Model config (`openpi/src/openpi/training/config.py:640-660`)

```python
TrainConfig(
    name="pi0_ur5",
    model=pi0_fast.Pi0FASTConfig(
        action_dim=8,          # 6 joints + 1 pad + 1 gripper
        action_horizon=10,     # 10 timesteps per chunk
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
    num_train_steps=10_000,
    freeze_filter=Pi0FASTConfig(...).get_freeze_filter(),
    ema_decay=None,
    batch_size=16,
    log_interval=500,
)
```

### 3. OpenPI FASTTokenizer.extract_actions() (`openpi/src/openpi/models/tokenizer.py:119-134`)

```python
def extract_actions(self, tokens, action_horizon, action_dim):
    decoded_tokens = self._paligemma_tokenizer.decode(tokens.tolist())
    if "Action: " not in decoded_tokens:
        return np.zeros((action_horizon, action_dim), dtype=np.float32)

    raw_action_tokens = np.array(
        self._paligemma_tokenizer.encode(
            decoded_tokens.split("Action: ")[1].split("|")[0].strip()
        )
    )
    action_tokens = self._act_tokens_to_paligemma_tokens(raw_action_tokens)
    return self._fast_tokenizer.decode(          # <-- calls HuggingFace processor
        [action_tokens.tolist()],
        time_horizon=action_horizon,             # 10
        action_dim=action_dim                    # 8
    )[0]
```

### 4. HuggingFace FAST processor decode (`~/.cache/huggingface/.../processing_action_tokenizer.py:60-96`)

This is where the error is raised:

```python
def decode(self, tokens, *, time_horizon=None, action_dim=None):
    ...
    for token in tokens:
        try:
            decoded_tokens = self.bpe_tokenizer.decode(token)
            decoded_dct_coeff = np.array(list(map(ord, decoded_tokens))) + self.min_token
            decoded_dct_coeff = decoded_dct_coeff.reshape(-1, self.action_dim)   # <-- FAILS HERE
            assert decoded_dct_coeff.shape == (self.time_horizon, self.action_dim)
        except Exception as e:
            print(f"Error decoding tokens: {e}")               # <-- error we see
            print(f"Tokens: {token}")
            decoded_dct_coeff = np.zeros((self.time_horizon, self.action_dim))   # <-- returns zeros
        decoded_actions.append(idct(decoded_dct_coeff / self.scale, axis=0, norm="ortho"))
    return np.stack(decoded_actions)
```

**Critical difference**: The LeRobot version of this same function (in `lerobot/common/policies/pi0fast/modeling_pi0fast.py:746-802`) has a `relaxed_decoding` flag that pads/truncates the DCT coefficient array to `time_horizon * action_dim` BEFORE reshape. The HuggingFace `UniversalActionProcessor` used by OpenPI does NOT have this. So any length mismatch → zeros.

---

## Root Cause Analysis

### Three contributing factors

**Factor 1: No relaxed decoding in the HuggingFace FAST tokenizer**

The HuggingFace `UniversalActionProcessor.decode()` does a hard `reshape(-1, action_dim)` followed by an exact shape assert. If the BPE decode produces any number of characters not equal to `time_horizon * action_dim = 80`, the decode fails and returns all zeros.

The LeRobot version has `relaxed_decoding=True` that handles this:
```python
if relaxed_decoding:
    expected_seq_len = self.time_horizon * self.action_dim  # 80
    diff = expected_seq_len - decoded_dct_coeff.shape[0]
    if diff < 0:
        decoded_dct_coeff = decoded_dct_coeff[:expected_seq_len]    # truncate
    elif diff > 0:
        decoded_dct_coeff = np.pad(decoded_dct_coeff, (0, diff))   # pad with 0
```

**Factor 2: BPE is inherently variable-rate**

The FAST tokenization pipeline:
1. Training: `action_chunk (10,8)` → DCT → 80 float coefficients → quantize → 80 characters → BPE encode → N tokens
2. Inference: model generates M tokens → BPE decode → K characters (K ≠ 80 if M ≠ N or if tokens are slightly wrong)

Even a well-trained model will occasionally produce token sequences that BPE-decode to the wrong number of characters. This is inherent to the FAST approach — the BPE compression means the model must learn to produce not just valid characters but valid *BPE-compressed* character sequences.

**Factor 3: Model still converging (training loss not plateaued)**

W&B `ur5_fast_v1` run shows:
- Loss at step 7.5k: ~0.19
- Loss at step 10k: ~0.13
- **Still decreasing** — the model hasn't converged yet
- CE loss of 0.13 means the model gets most tokens right, but FAST tokenization is brittle: even 1-2 wrong tokens can produce the wrong number of BPE-decoded characters

The 100% failure rate is consistent with the model being "almost good enough" but not yet precise enough for the strict (non-relaxed) decode path.

---

## Training Details

| Parameter | Value |
|---|---|
| Config name | `pi0_ur5` |
| Model | `Pi0FASTConfig` (pi0-FAST with LoRA) |
| Base checkpoint | `gs://openpi-assets/checkpoints/pi0_fast_base/params` |
| PaliGemma variant | `gemma_2b_lora` |
| action_dim | 8 (6 joints + 1 zero pad + 1 gripper) |
| action_horizon | 10 |
| max_token_len | 180 |
| Dataset | `sheilsarda/ur5_isaac_sim_v1` (LeRobot format) |
| Batch size | 16 |
| Training steps completed | 10,000 |
| Final CE loss | ~0.13 (still decreasing) |
| Checkpoint path | `checkpoints/pi0_ur5/ur5_fast_v1/9999` |
| HuggingFace checkpoint | `sheilsarda/pi0_ur5_fast_v1` |
| W&B run | `dtoe5fhc` |

---

## Fix Plan

### Phase 1: Patch relaxed decoding (immediate — unblocks evaluation)

Patch `UniversalActionProcessor.decode()` in the HuggingFace cache to add truncation/padding before reshape. This lets approximate token sequences produce approximate actions instead of all zeros.

**File to patch:**
```
~/.cache/huggingface/modules/transformers_modules/physical-intelligence/fast/
  ec4d7aa71691cac0b8bed6942be45684db2110f4/processing_action_tokenizer.py
```

**Change** (line 83, inside the `try` block, BEFORE the reshape):

```python
# --- BEFORE (current code) ---
decoded_dct_coeff = decoded_dct_coeff.reshape(-1, self.action_dim)

# --- AFTER (with relaxed decoding) ---
expected_seq_len = self.time_horizon * self.action_dim
diff = expected_seq_len - decoded_dct_coeff.shape[0]
if diff < 0:
    decoded_dct_coeff = decoded_dct_coeff[:expected_seq_len]
elif diff > 0:
    decoded_dct_coeff = np.pad(decoded_dct_coeff, (0, diff), mode="constant", constant_values=0)
decoded_dct_coeff = decoded_dct_coeff.reshape(-1, self.action_dim)
```

This matches what LeRobot's `modeling_pi0fast.py:777-788` does with `relaxed_decoding=True`.

**Risk**: Truncation/padding of DCT coefficients introduces approximation error. High-frequency DCT components at the end of the sequence are least important (DCT is ordered by frequency), so truncation is relatively benign. Padding with zeros is equivalent to assuming zero high-frequency components.

### Phase 2: Resume training (to convergence)

```bash
cd ~/Development/openpi
uv run scripts/train.py pi0_ur5 --exp-name=ur5_fast_v1 --resume --num-train-steps 30000
```

- The loss was still clearly decreasing at 10k steps
- Target: train until loss plateaus (expected around 20-30k steps)
- Watch W&B for `loss` flattening — that's the convergence signal
- Overfitting risk is low: LoRA constrains capacity, and the dataset has enough variety for this task
- If loss doesn't plateau by 30k, consider increasing batch size or adjusting learning rate

### Phase 3: Evaluate model quality

Once relaxed decoding is in place, we can evaluate whether the model produces useful (even if approximate) actions:

1. **Run with `dry_run: true`** — infer actions but don't send to arm. Log action chunks in the VLA controller node output.
2. **Check action magnitudes** — are the delta joint values reasonable (small, in radians)?
3. **Check action consistency** — do successive inferences for the same observation produce similar actions?
4. **Run live** — with the arm in Isaac Sim, does it move toward the task goal?

### Phase 4: Offline rosbag analysis

Record controller output for offline visualization:

```bash
ros2 bag record \
  /joint_states \
  /ur_manipulator_controller/follow_joint_trajectory/_action/send_goal \
  /camera/image_raw \
  /camera_wrist/image_raw \
  -o vla_eval_bag
```

Visualization options:
- **PlotJuggler** (`ros-jazzy-plotjuggler-ros`) — plot joint trajectories from bags, compare commanded vs actual
- **Custom Python script** — use `rosbag2_py` to extract `/joint_states` and commanded trajectories, plot with matplotlib
- **Foxglove Studio** — web-based replay with synchronized camera feeds + joint plots

What to look for:
- Are commanded joint deltas reasonable in magnitude?
- Do the joints drift or oscillate?
- Does the arm move toward the object?
- Compare camera view with commanded motion direction

---

## System Architecture Reference

```
┌──────────────────────────────────────────────────────────────────┐
│  Isaac Sim                                                       │
│  ├── /camera/image_raw         (224×224 RGB)                     │
│  ├── /camera_wrist/image_raw   (224×224 RGB)                     │
│  └── /joint_states             (6 joints + gripper)              │
└───────────────────────────────┬──────────────────────────────────┘
                                │
                   ROS2 Topics  │
                                │
┌───────────────────────────────▼──────────────────────────────────┐
│  vla_controller_node (ROS2)                                      │
│                                                                  │
│  1. Subscribe to images + joint_states                           │
│  2. Build observation dict:                                      │
│     - observation/exterior_image_1_left  (224,224,3) uint8       │
│     - observation/wrist_image_left       (224,224,3) uint8       │
│     - observation/joint_position         (6,) float32            │
│     - observation/gripper_position       (1,) float32            │
│     - prompt                             str                     │
│  3. Send via websocket (msgpack + numpy serialization)           │
│  4. Receive action chunk (action_horizon, action_dim) float32    │
│  5. Slice to [:, :6], apply as delta joint angles                │
│  6. Send JointTrajectory goal to controller_manager              │
└───────────────────────────────┬──────────────────────────────────┘
                                │
                   WebSocket    │  ws://localhost:8000
                                │
┌───────────────────────────────▼──────────────────────────────────┐
│  OpenPI Inference Server                                         │
│                                                                  │
│  Input transforms (Ur5Inputs):                                   │
│    - Pad joint_position (6,) → (7,) with zero                   │
│    - Concat gripper → state (8,)                                │
│    - Map images to base_0_rgb, base_1_rgb, wrist_0_rgb          │
│                                                                  │
│  Model (pi0-FAST):                                               │
│    - SigLIP image encoder                                        │
│    - PaliGemma 2B (LoRA) language model                         │
│    - Autoregressive token generation                             │
│                                                                  │
│  Output transforms:                                              │
│    - ExtractFASTActions: tokens → FAST decode → (10, 8) actions │
│    - Unnormalize actions                                         │
│    - Ur5Outputs: slice to [:, :7] (6 joints + gripper)          │
│                                                                  │
│  *** FAILURE POINT ***                                           │
│  ExtractFASTActions calls UniversalActionProcessor.decode()      │
│  which does BPE decode → char array → reshape(-1, 8)            │
│  Reshape fails when char count ≠ 80                             │
│  Falls back to zeros → arm doesn't move                         │
└──────────────────────────────────────────────────────────────────┘
```

---

## Key File Locations

| File | Purpose |
|---|---|
| `openpi/src/openpi/training/config.py:640-660` | `pi0_ur5` TrainConfig |
| `openpi/src/openpi/models/pi0_fast.py:77-83` | Pi0FASTConfig defaults (action_dim=32, action_horizon=32) |
| `openpi/src/openpi/models/tokenizer.py:51-139` | FASTTokenizer (openpi's wrapper) |
| `openpi/src/openpi/policies/ur5_policy.py` | Ur5Inputs / Ur5Outputs transforms |
| `openpi/src/openpi/transforms.py:292-306` | ExtractFASTActions output transform |
| `~/.cache/huggingface/.../processing_action_tokenizer.py` | HuggingFace FAST processor (error site) |
| `openpi/.venv/.../pi0fast/modeling_pi0fast.py:746-802` | LeRobot version with relaxed_decoding |
| `VLA_Arm_Controller/src/vla_controller/vla_controller/vla_controller_node.py` | ROS2 bridge node |
| `VLA_Arm_Controller/src/vla_controller/config/vla_params.yaml` | Bridge parameters |

---

## Open Questions

1. **SentencePiece round-trip fidelity**: Does `paligemma.encode(paligemma.decode(tokens))` round-trip losslessly for FAST action tokens? If not, this could be corrupting tokens even when the model generates them correctly. Worth testing with a known-good token sequence from training data.

2. **Is 10k steps the right order of magnitude?** The DROID FAST configs in `polaris_config.py` also train for ~1k steps but with batch_size=128 (8x more samples per step). Our effective sample count is `10k * 16 = 160k` vs their `1k * 128 = 128k` — similar, but they start from a DROID-pretrained checkpoint while we start from `pi0_fast_base`.

3. **Dataset quality**: How many episodes are in `sheilsarda/ur5_isaac_sim_v1`? If it's a small dataset (<50 episodes), the model may struggle to generalize the BPE structure. More diverse demonstrations would help.

4. **Alternative: pi0-base (non-FAST)**: The `pi0_ur5_base` config uses flow-matching decoding instead of FAST tokenization. This avoids the BPE decode fragility entirely but has higher inference latency. Could be a more robust fallback if FAST continues to be unreliable.
