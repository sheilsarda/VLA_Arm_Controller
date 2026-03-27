"""Analyze training episode rosbags to extract ground-truth joint deltas.

Reads the demonstration rosbags used to train the pi0-FAST model, computes
per-timestep joint deltas, generates summary statistics and plots, and appends
a "Training Data Analysis" section to the eval datadump markdown file.

Usage (run from the VLA_Arm_Controller .venv):
    .venv/bin/python scripts/analyze_training_episodes.py
"""

import os
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
from mcap_ros2.reader import read_ros2_messages

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
]

EXPECTED_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

_PROJECT_ROOT = os.path.expanduser("~/Development/VLA_Arm_Controller")
TRAINING_DATA_DIR = os.path.join(_PROJECT_ROOT, "training_data")
THOUGHTS_DIR = os.path.join(_PROJECT_ROOT, "thoughts")
MARKDOWN_FILE = os.path.join(
    THOUGHTS_DIR, "Model Finetuning Eval Datadump 032226.md"
)

# Eval run waypoint interval — used to normalize training deltas (recorded at
# ~35 Hz) to the same timescale as eval actions (10 Hz / 0.1s per step).
EVAL_WAYPOINT_DT = 0.1


def _resolve_joint_indices(joint_names):
    """Build a mapping from our expected joint order to the message's order."""
    name_to_idx = {name: i for i, name in enumerate(joint_names)}
    indices = []
    for name in EXPECTED_JOINT_NAMES:
        if name not in name_to_idx:
            return None
        indices.append(name_to_idx[name])
    return indices


def _read_raw_joint_states(episode_dir: str):
    """Read all raw /joint_states messages from an episode bag, no resampling.

    Returns (timestamps, positions) where timestamps is shape (N,) in seconds
    and positions is shape (N, 6), one row per message as recorded.
    """
    mcap_files = [f for f in os.listdir(episode_dir) if f.endswith(".mcap")]
    if not mcap_files:
        raise FileNotFoundError(f"No .mcap file in {episode_dir}")
    mcap_path = os.path.join(episode_dir, mcap_files[0])

    joint_index_map = None
    first_timestamp = None
    timestamps = []
    positions = []

    for msg_view in read_ros2_messages(mcap_path, topics=["/joint_states"]):
        msg = msg_view.ros_msg
        timestamp_ns = msg_view.log_time_ns

        if joint_index_map is None:
            joint_index_map = _resolve_joint_indices(list(msg.name))
            if joint_index_map is None:
                continue

        if first_timestamp is None:
            first_timestamp = timestamp_ns

        t = (timestamp_ns - first_timestamp) / 1e9
        pos = np.array(msg.position)[joint_index_map]
        timestamps.append(t)
        positions.append(pos)

    return np.array(timestamps), np.stack(positions)


def read_episode_joint_states(episode_dir: str, resample_dt: float = 0.1):
    """Read joint states from a training episode rosbag, resampled to a fixed rate.

    Returns (timestamps, positions) where timestamps is shape (N,) in seconds
    relative to the first message and positions is shape (N, 6).
    """
    raw_ts, raw_pos = _read_raw_joint_states(episode_dir)

    # Resample to fixed interval using nearest-neighbor lookup.
    duration = raw_ts[-1]
    sample_times = np.arange(0, duration, resample_dt)
    indices = np.searchsorted(raw_ts, sample_times, side="right") - 1
    indices = np.clip(indices, 0, len(raw_ts) - 1)

    return sample_times, raw_pos[indices]


def load_all_episodes():
    """Load joint states from all training episodes.

    Returns a dict with per-episode data and aggregate arrays.
    """
    episodes = {}
    all_deltas_list = []

    for i in range(1, 11):
        ep_dir = os.path.join(TRAINING_DATA_DIR, f"episode_{i:03d}")
        if not os.path.isdir(ep_dir):
            print(f"Skipping {ep_dir} (not found)")
            continue

        print(f"Reading {ep_dir}...")
        timestamps, positions = read_episode_joint_states(ep_dir)

        # Deltas between resampled positions (already at 0.1s intervals,
        # matching the eval waypoint cadence — no normalization needed).
        deltas = np.diff(positions, axis=0)
        mid_times = (timestamps[:-1] + timestamps[1:]) / 2

        episodes[i] = {
            "timestamps": timestamps,
            "positions": positions,
            "deltas": deltas,
            "mid_times": mid_times,
        }
        all_deltas_list.append(deltas)

    all_deltas = np.concatenate(all_deltas_list, axis=0)
    return episodes, all_deltas


def generate_plots(episodes, all_deltas):
    """Generate and save plots to the thoughts/ directory."""

    # --- Per-episode plots: raw joint positions ---
    ep_plots_dir = os.path.join(THOUGHTS_DIR, "episode_plots")
    os.makedirs(ep_plots_dir, exist_ok=True)

    for ep_id in sorted(episodes.keys()):
        ep_dir = os.path.join(TRAINING_DATA_DIR, f"episode_{ep_id:03d}")
        raw_ts, raw_pos = _read_raw_joint_states(ep_dir)
        fig, axes = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(f"Episode {ep_id:03d} — Raw Joint Positions Over Time", fontsize=14)
        for i, ax in enumerate(axes):
            ax.plot(raw_ts, raw_pos[:, i], "b-", linewidth=0.5)
            ax.set_ylabel(f"{JOINT_NAMES[i]}\n(rad)", fontsize=8)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("Time (s)")
        fig.tight_layout()
        fig.savefig(os.path.join(ep_plots_dir, f"episode_{ep_id:03d}.png"), dpi=150, bbox_inches="tight")
        plt.close(fig)

    print(f"Saved per-episode plots to {ep_plots_dir}/")

    # --- Plot 1: Joint positions over time (all episodes overlaid) ---
    fig1, axes1 = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
    fig1.suptitle(
        "Training Data — Joint Positions Over Time (all episodes)", fontsize=14
    )
    colors = plt.cm.tab10(np.linspace(0, 1, len(episodes)))

    for (ep_id, ep_data), color in zip(
        sorted(episodes.items()), colors
    ):
        for i, ax in enumerate(axes1):
            ax.scatter(
                ep_data["timestamps"],
                ep_data["positions"][:, i],
                s=2,
                alpha=0.5,
                color=color,
                label=f"ep {ep_id}" if i == 0 else None,
            )

    for i, ax in enumerate(axes1):
        ax.set_ylabel(f"{JOINT_NAMES[i]}\n(rad)", fontsize=8)
        ax.grid(True, alpha=0.3)
    axes1[0].legend(loc="upper right", fontsize=6, ncol=5)
    axes1[-1].set_xlabel("Time (s)")
    fig1.tight_layout()
    fig1.savefig(
        os.path.join(THOUGHTS_DIR, "training_data_joint_positions.png"),
        dpi=150,
        bbox_inches="tight",
    )

    # --- Plot 2: Delta distributions (boxplot + histogram) ---
    fig2, (ax_box, ax_hist) = plt.subplots(1, 2, figsize=(12, 5))
    fig2.suptitle(
        "Training Data — Delta Distributions (0.1s steps)",
        fontsize=14,
    )

    ax_box.boxplot(
        [all_deltas[:, i] for i in range(6)],
        tick_labels=JOINT_NAMES,
        showfliers=False,  # hide extreme outliers that compress the y-axis
    )
    ax_box.set_ylabel("Delta (rad)")
    ax_box.set_title("Delta Distribution per Joint (outliers hidden)")
    ax_box.axhline(y=0, color="k", linewidth=0.5, linestyle="--")
    ax_box.grid(True, alpha=0.3)

    magnitudes = np.linalg.norm(all_deltas, axis=1)
    # Clip to 99th percentile so outliers don't compress the histogram.
    p99 = np.percentile(magnitudes, 99)
    clipped = magnitudes[magnitudes <= p99]
    ax_hist.hist(clipped, bins=50, edgecolor="black", alpha=0.7)
    ax_hist.set_xlabel("L2 norm of delta vector")
    ax_hist.set_ylabel("Count")
    ax_hist.set_title(f"Delta Magnitude Distribution (clipped to p99={p99:.4f})")
    ax_hist.axvline(
        x=np.median(magnitudes),
        color="r",
        linestyle="--",
        label=f"median={np.median(magnitudes):.4f}",
    )
    ax_hist.legend()
    ax_hist.grid(True, alpha=0.3)

    fig2.tight_layout()
    fig2.savefig(
        os.path.join(THOUGHTS_DIR, "training_data_delta_distributions.png"),
        dpi=150,
        bbox_inches="tight",
    )

    # --- Plot 3: Deltas over time (per episode) ---
    fig3, axes3 = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
    fig3.suptitle(
        "Training Data — Joint Deltas Over Time (0.1s steps)",
        fontsize=14,
    )

    for (ep_id, ep_data), color in zip(
        sorted(episodes.items()), colors
    ):
        for i, ax in enumerate(axes3):
            ax.plot(
                ep_data["mid_times"],
                ep_data["deltas"][:, i],
                linewidth=0.5,
                alpha=0.6,
                color=color,
            )

    for i, ax in enumerate(axes3):
        ax.set_ylabel(f"{JOINT_NAMES[i]}\n(delta rad)", fontsize=8)
        ax.axhline(y=0, color="k", linewidth=0.5, linestyle="--")
        ax.grid(True, alpha=0.3)
    axes3[-1].set_xlabel("Time (s)")
    fig3.tight_layout()
    fig3.savefig(
        os.path.join(THOUGHTS_DIR, "training_data_delta_timeseries.png"),
        dpi=150,
        bbox_inches="tight",
    )

    plt.show()

    return magnitudes


_SECTION_MARKER = "## Training Data Analysis — Ground-Truth Demonstration Deltas"


def append_to_markdown(episodes, all_deltas, magnitudes):
    """Write the Training Data Analysis section to the datadump markdown.

    If the section already exists, replaces it in-place. Otherwise appends.
    """

    total_msgs = sum(
        ep["positions"].shape[0] for ep in episodes.values()
    )
    total_deltas = all_deltas.shape[0]
    n_episodes = len(episodes)

    # Per-joint stats.
    means = np.mean(all_deltas, axis=0)
    stds = np.std(all_deltas, axis=0)

    # Eval Run 2 data for comparison table.
    eval2_means = np.array(
        [0.00067263, -0.00045258, 0.00027294, -0.00470189, -0.0036581, 0.00119332]
    )
    eval2_stds = np.array(
        [0.00656161, 0.00420713, 0.00212963, 0.00973055, 0.00714096, 0.00422311]
    )

    section = f"""

---

## Training Data Analysis — Ground-Truth Demonstration Deltas

**Date:** {datetime.now().strftime('%Y-%m-%d')}
**Episodes:** episode_001 through episode_{n_episodes:03d} ({n_episodes} total)
**Task:** "lift the arm up"
**Total Samples (resampled at 0.1s):** {total_msgs:,}
**Total Delta Samples:** {total_deltas:,}

> Positions are resampled from raw bag data to 0.1s intervals via nearest-neighbor
> lookup. This matches the eval waypoint cadence, so deltas are directly comparable
> without normalization.

### Per-Joint Delta Statistics (across all {n_episodes} episodes)

| Joint | Mean Delta (rad) | Std (rad) |
|---|---|---|
| shoulder_pan | {means[0]:.6f} | {stds[0]:.6f} |
| shoulder_lift | {means[1]:.6f} | {stds[1]:.6f} |
| elbow | {means[2]:.6f} | {stds[2]:.6f} |
| wrist_1 | {means[3]:.6f} | {stds[3]:.6f} |
| wrist_2 | {means[4]:.6f} | {stds[4]:.6f} |
| wrist_3 | {means[5]:.6f} | {stds[5]:.6f} |

**Magnitude summary:** median={np.median(magnitudes):.4f}, mean={np.mean(magnitudes):.4f}, max={np.max(magnitudes):.4f}

### Comparison: Training Deltas vs Model Output (Eval Run 2)

| Joint | Training Mean | Training Std | Eval Run 2 Mean | Eval Run 2 Std | Mean Ratio |
|---|---|---|---|---|---|
| shoulder_pan | {means[0]:.6f} | {stds[0]:.6f} | {eval2_means[0]:.6f} | {eval2_stds[0]:.6f} | {eval2_means[0]/means[0]:.1f}x |
| shoulder_lift | {means[1]:.6f} | {stds[1]:.6f} | {eval2_means[1]:.6f} | {eval2_stds[1]:.6f} | {eval2_means[1]/means[1]:.1f}x |
| elbow | {means[2]:.6f} | {stds[2]:.6f} | {eval2_means[2]:.6f} | {eval2_stds[2]:.6f} | {eval2_means[2]/means[2]:.1f}x |
| wrist_1 | {means[3]:.6f} | {stds[3]:.6f} | {eval2_means[3]:.6f} | {eval2_stds[3]:.6f} | {eval2_means[3]/means[3]:.1f}x |
| wrist_2 | {means[4]:.6f} | {stds[4]:.6f} | {eval2_means[4]:.6f} | {eval2_stds[4]:.6f} | {eval2_means[4]/means[4]:.1f}x |
| wrist_3 | {means[5]:.6f} | {stds[5]:.6f} | {eval2_means[5]:.6f} | {eval2_stds[5]:.6f} | {eval2_means[5]/means[5]:.1f}x |

### Plots

![Training Data Joint Positions](training_data_joint_positions.png)

![Training Data Delta Distributions](training_data_delta_distributions.png)

![Training Data Delta Time Series](training_data_delta_timeseries.png)
"""

    with open(MARKDOWN_FILE, "r") as f:
        content = f.read()

    if _SECTION_MARKER in content:
        # Replace everything from the marker to the end of the file.
        idx = content.index(_SECTION_MARKER)
        # Walk back to include the preceding "---" separator.
        prefix = content[:idx].rstrip()
        if prefix.endswith("---"):
            prefix = prefix[: -len("---")].rstrip()
        content = prefix + section
    else:
        content += section

    with open(MARKDOWN_FILE, "w") as f:
        f.write(content)

    print(f"Wrote Training Data Analysis section to {MARKDOWN_FILE}")


def main():
    episodes, all_deltas = load_all_episodes()

    if not episodes:
        print("No episodes found. Exiting.")
        return

    # Print summary to console.
    means = np.mean(all_deltas, axis=0)
    stds = np.std(all_deltas, axis=0)
    magnitudes_arr = np.linalg.norm(all_deltas, axis=1)

    print(f"\n=== Training Data Delta Summary (normalized to 0.1s steps) ===")
    print(f"Episodes: {len(episodes)}")
    print(f"Total delta samples: {all_deltas.shape[0]}")
    print(
        f"Delta magnitude — median: {np.median(magnitudes_arr):.4f}, "
        f"mean: {np.mean(magnitudes_arr):.4f}, max: {np.max(magnitudes_arr):.4f}"
    )
    print(f"Per-joint mean deltas: {means}")
    print(f"Per-joint std deltas:  {stds}")

    magnitudes_arr = generate_plots(episodes, all_deltas)
    append_to_markdown(episodes, all_deltas, magnitudes_arr)

    print("Done.")


if __name__ == "__main__":
    main()
