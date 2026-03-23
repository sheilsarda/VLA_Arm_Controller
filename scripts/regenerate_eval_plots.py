"""Regenerate eval run plots from saved rosbag using scatter plots.

Usage:
    .venv/bin/python scripts/regenerate_eval_plots.py training_data/eval_runs/eval_20260322_161508
"""

import os
import sys

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
THOUGHTS_DIR = os.path.join(_PROJECT_ROOT, "thoughts")


def main():
    if len(sys.argv) < 2:
        print("Usage: python scripts/regenerate_eval_plots.py <bag_dir>")
        sys.exit(1)

    bag_dir = sys.argv[1]
    if not os.path.isabs(bag_dir):
        bag_dir = os.path.join(_PROJECT_ROOT, bag_dir)

    mcap_files = [f for f in os.listdir(bag_dir) if f.endswith(".mcap")]
    if not mcap_files:
        print(f"No .mcap file in {bag_dir}")
        sys.exit(1)
    mcap_path = os.path.join(bag_dir, mcap_files[0])

    # Read all data from bag.
    joint_index_map = None
    first_ts = None
    joint_times, joint_positions = [], []
    raw_actions = []  # (t, np.array(n_steps, 6))
    predicted_targets = []  # (t, np.array(n_steps, 6))

    for msg_view in read_ros2_messages(mcap_path):
        topic = msg_view.channel.topic
        msg = msg_view.ros_msg
        ts_ns = msg_view.log_time_ns

        if first_ts is None:
            first_ts = ts_ns
        t = (ts_ns - first_ts) / 1e9

        if topic == "/joint_states":
            if joint_index_map is None:
                name_to_idx = {n: i for i, n in enumerate(msg.name)}
                joint_index_map = []
                for name in EXPECTED_JOINT_NAMES:
                    if name not in name_to_idx:
                        joint_index_map = None
                        continue
                    joint_index_map.append(name_to_idx[name])
                if joint_index_map is None:
                    continue
            pos = np.array(msg.position)[joint_index_map]
            joint_times.append(t)
            joint_positions.append(pos)

        elif topic == "/vla_controller_node/raw_actions":
            data = np.array(msg.data)
            n_steps = len(data) // 6
            if n_steps > 0:
                raw_actions.append((t, data.reshape(n_steps, 6)))

        elif topic == "/vla_controller_node/predicted_targets":
            data = np.array(msg.data)
            n_steps = len(data) // 6
            if n_steps > 0:
                predicted_targets.append((t, data.reshape(n_steps, 6)))

    times = np.array(joint_times)
    positions = np.stack(joint_positions)

    print(f"Data: {len(times)} joint states, {len(raw_actions)} action chunks, "
          f"{len(predicted_targets)} target chunks")

    # --- Figure 1: Joint positions over time (scatter) ---
    fig1, axes1 = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
    fig1.suptitle("Joint Positions Over Time", fontsize=14)

    for i, ax in enumerate(axes1):
        ax.scatter(times, positions[:, i], s=1, alpha=0.5, c="blue", label="actual")

        if predicted_targets:
            for t, tgt in predicted_targets:
                t_points = t + np.arange(tgt.shape[0]) * 0.1
                ax.scatter(t_points, tgt[:, i], s=8, c="red", alpha=0.7, marker=".")

            ax.scatter([], [], s=8, c="red", marker=".", label="predicted target")

        ax.set_ylabel(f"{JOINT_NAMES[i]}\n(rad)", fontsize=8)
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)

    axes1[-1].set_xlabel("Time (s)")
    fig1.tight_layout()
    fig1.savefig(
        os.path.join(THOUGHTS_DIR, "model_eval_jointpositionsovertime.png"),
        dpi=150, bbox_inches="tight",
    )

    # --- Figure 2: Raw action deltas per chunk ---
    if raw_actions:
        fig2, axes2 = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
        fig2.suptitle("Raw Action Deltas (per inference chunk)", fontsize=14)

        for i, ax in enumerate(axes2):
            for chunk_idx, (t, act) in enumerate(raw_actions):
                step_times = t + np.arange(act.shape[0]) * 0.1
                ax.scatter(step_times, act[:, i], s=8, alpha=0.7, marker=".")

            ax.set_ylabel(f"{JOINT_NAMES[i]}\n(delta rad)", fontsize=8)
            ax.axhline(y=0, color="k", linewidth=0.5, linestyle="--")
            ax.grid(True, alpha=0.3)

        axes2[-1].set_xlabel("Time (s)")
        fig2.tight_layout()
        fig2.savefig(
            os.path.join(THOUGHTS_DIR, "model_eval_rawactiondeltas.png"),
            dpi=150, bbox_inches="tight",
        )

    # --- Figure 3: Action magnitude summary ---
    if raw_actions:
        fig3, (ax_mag, ax_hist) = plt.subplots(1, 2, figsize=(12, 5))
        fig3.suptitle("Action Magnitude Analysis", fontsize=14)

        all_actions = np.concatenate([act for _, act in raw_actions], axis=0)

        ax_mag.boxplot(
            [all_actions[:, i] for i in range(6)],
            tick_labels=JOINT_NAMES,
        )
        ax_mag.set_ylabel("Delta (rad)")
        ax_mag.set_title("Action Distribution per Joint")
        ax_mag.axhline(y=0, color="k", linewidth=0.5, linestyle="--")
        ax_mag.grid(True, alpha=0.3)

        magnitudes = np.linalg.norm(all_actions, axis=1)
        ax_hist.hist(magnitudes, bins=30, edgecolor="black", alpha=0.7)
        ax_hist.set_xlabel("L2 norm of action vector")
        ax_hist.set_ylabel("Count")
        ax_hist.set_title("Action Magnitude Distribution")
        ax_hist.axvline(
            x=np.median(magnitudes), color="r", linestyle="--",
            label=f"median={np.median(magnitudes):.4f}",
        )
        ax_hist.legend()
        ax_hist.grid(True, alpha=0.3)

        fig3.tight_layout()
        fig3.savefig(
            os.path.join(THOUGHTS_DIR, "model_eval_actionmagnitude.png"),
            dpi=150, bbox_inches="tight",
        )

        print(f"\n=== Action Summary ===")
        print(f"Total chunks: {len(raw_actions)}")
        print(f"Total action steps: {all_actions.shape[0]}")
        print(f"Action magnitude — median: {np.median(magnitudes):.4f}, "
              f"mean: {np.mean(magnitudes):.4f}, max: {np.max(magnitudes):.4f}")
        print(f"Per-joint mean deltas: {np.mean(all_actions, axis=0)}")
        print(f"Per-joint std deltas:  {np.std(all_actions, axis=0)}")

    plt.show()
    print(f"\nPlots saved to {THOUGHTS_DIR}/model_eval_*.png")


if __name__ == "__main__":
    main()
