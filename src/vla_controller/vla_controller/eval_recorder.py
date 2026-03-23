"""Record and visualize VLA controller evaluation runs.

Records a rosbag of diagnostic topics while the VLA controller runs (live or
dry-run mode), then produces matplotlib plots on Ctrl+C (or after a timeout).
Bags are saved to training_data/eval_runs/ for later replay.

Usage:
    # Live recording + plotting (Ctrl+C to stop and plot):
    ros2 run vla_controller eval_recorder

    # Auto-stop after 60 seconds:
    ros2 run vla_controller eval_recorder --ros-args -p timeout_sec:=60.0

    # Replay a previously recorded bag (no live subscription needed):
    ros2 run vla_controller eval_recorder --ros-args -p replay:=training_data/eval_runs/eval_20260322_154800
"""

import os
import signal
import sys
import time
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow",
    "wrist_1",
    "wrist_2",
    "wrist_3",
]

# Joints in /joint_states may appear in arbitrary order; these are the names to look for.
EXPECTED_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

TOPICS_TO_RECORD = [
    ("/joint_states", "sensor_msgs/msg/JointState"),
    ("/vla_controller_node/raw_actions", "std_msgs/msg/Float64MultiArray"),
    ("/vla_controller_node/predicted_targets", "std_msgs/msg/Float64MultiArray"),
]

# Hardcode project root — relative paths don't work because ROS2 runs from the
# install/ tree, not the source tree.
_PROJECT_ROOT = os.path.expanduser("~/Development/VLA_Arm_Controller")
EVAL_DIR = os.path.join(_PROJECT_ROOT, "training_data", "eval_runs")


def _make_bag_path() -> str:
    """Return a timestamped path for a new evaluation rosbag.

    Only creates the parent eval_runs/ directory — the bag directory itself is
    created by the rosbag2 writer (it errors if the directory already exists).
    """
    os.makedirs(EVAL_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(EVAL_DIR, f"eval_{stamp}")


class EvalRecorderNode(Node):
    def __init__(self):
        super().__init__("eval_recorder")

        self.declare_parameter("timeout_sec", 0.0)
        self.declare_parameter("replay", "")
        self.timeout_sec = self.get_parameter("timeout_sec").value
        self.replay_path = str(self.get_parameter("replay").value)

        # Accumulated data: list of (timestamp, values) tuples.
        self.joint_positions = []  # (t, np.array(6,))
        self.raw_actions = []  # (t, np.array(n_steps, 6))
        self.predicted_targets = []  # (t, np.array(n_steps, 6))

        self._start_time = time.monotonic()
        self._joint_index_map = None

        if self.replay_path:
            # Replay mode: load data from a saved bag and plot immediately.
            self._replay_and_plot()
            return

        # Live mode: subscribe to topics and record a rosbag.
        self._bag_path = _make_bag_path()
        self._init_bag_writer()

        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 30
        )
        self.create_subscription(
            Float64MultiArray,
            "/vla_controller_node/raw_actions",
            self._raw_actions_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/vla_controller_node/predicted_targets",
            self._predicted_targets_cb,
            10,
        )

        if self.timeout_sec > 0:
            self.create_timer(1.0, self._check_timeout)

        self.get_logger().info(
            f"eval_recorder started. Recording to {self._bag_path}\n"
            f"{'Press Ctrl+C to stop and plot.' if self.timeout_sec <= 0 else f'Will auto-plot after {self.timeout_sec}s.'}"
        )

    # ---- Rosbag writer ----

    def _init_bag_writer(self):
        import rosbag2_py

        self._writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=self._bag_path, storage_id="mcap"
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        self._writer.open(storage_options, converter_options)

        for idx, (topic_name, topic_type) in enumerate(TOPICS_TO_RECORD):
            topic_meta = rosbag2_py.TopicMetadata(
                id=idx,
                name=topic_name,
                type=topic_type,
                serialization_format="cdr",
            )
            self._writer.create_topic(topic_meta)

    def _write_to_bag(self, topic: str, msg):
        import rosbag2_py

        serialized = serialize_message(msg)
        timestamp_ns = self.get_clock().now().nanoseconds
        self._writer.write(topic, serialized, timestamp_ns)

    # ---- Callbacks ----

    def _resolve_joint_indices(self, msg: JointState):
        """Build a mapping from our expected joint order to the message's joint order."""
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        indices = []
        for name in EXPECTED_JOINT_NAMES:
            if name not in name_to_idx:
                return None
            indices.append(name_to_idx[name])
        return indices

    def _joint_state_cb(self, msg: JointState):
        if self._joint_index_map is None:
            self._joint_index_map = self._resolve_joint_indices(msg)
            if self._joint_index_map is None:
                return

        self._write_to_bag("/joint_states", msg)

        positions = np.array(msg.position)
        ordered = positions[self._joint_index_map]
        t = time.monotonic() - self._start_time
        self.joint_positions.append((t, ordered))

    def _raw_actions_cb(self, msg: Float64MultiArray):
        self._write_to_bag("/vla_controller_node/raw_actions", msg)

        t = time.monotonic() - self._start_time
        data = np.array(msg.data)
        n_steps = len(data) // 6
        if n_steps > 0:
            self.raw_actions.append((t, data.reshape(n_steps, 6)))

    def _predicted_targets_cb(self, msg: Float64MultiArray):
        self._write_to_bag("/vla_controller_node/predicted_targets", msg)

        t = time.monotonic() - self._start_time
        data = np.array(msg.data)
        n_steps = len(data) // 6
        if n_steps > 0:
            self.predicted_targets.append((t, data.reshape(n_steps, 6)))

    def _check_timeout(self):
        if time.monotonic() - self._start_time >= self.timeout_sec:
            self.get_logger().info("Timeout reached, generating plots...")
            self._finalize()

    # ---- Replay from bag ----

    def _replay_and_plot(self):
        """Load data from a previously recorded rosbag and plot."""
        import rosbag2_py

        bag_path = self.replay_path
        if not os.path.exists(bag_path):
            # Try relative to project root.
            bag_path = os.path.join(_PROJECT_ROOT, self.replay_path)

        self.get_logger().info(f"Replaying from {bag_path}")

        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader.open(storage_options, converter_options)

        first_timestamp = None
        while reader.has_next():
            topic, serialized, timestamp_ns = reader.read_next()

            if first_timestamp is None:
                first_timestamp = timestamp_ns
            t = (timestamp_ns - first_timestamp) / 1e9

            if topic == "/joint_states":
                msg = deserialize_message(serialized, JointState)
                if self._joint_index_map is None:
                    self._joint_index_map = self._resolve_joint_indices(msg)
                    if self._joint_index_map is None:
                        continue
                positions = np.array(msg.position)
                ordered = positions[self._joint_index_map]
                self.joint_positions.append((t, ordered))

            elif topic == "/vla_controller_node/raw_actions":
                msg = deserialize_message(serialized, Float64MultiArray)
                data = np.array(msg.data)
                n_steps = len(data) // 6
                if n_steps > 0:
                    self.raw_actions.append((t, data.reshape(n_steps, 6)))

            elif topic == "/vla_controller_node/predicted_targets":
                msg = deserialize_message(serialized, Float64MultiArray)
                data = np.array(msg.data)
                n_steps = len(data) // 6
                if n_steps > 0:
                    self.predicted_targets.append((t, data.reshape(n_steps, 6)))

        self.plot()

    # ---- Finalize and plot ----

    def _finalize(self):
        """Close the bag writer and generate plots."""
        if hasattr(self, "_writer"):
            del self._writer
            self.get_logger().info(f"Rosbag saved to {self._bag_path}")
        self.plot()
        rclpy.shutdown()

    def plot(self):
        n_joints = len(self.joint_positions)
        n_actions = len(self.raw_actions)
        n_targets = len(self.predicted_targets)

        self.get_logger().info(
            f"Data collected: {n_joints} joint states, "
            f"{n_actions} action chunks, {n_targets} target chunks"
        )

        if n_joints == 0:
            self.get_logger().warn("No joint state data received. Nothing to plot.")
            return

        # --- Figure 1: Current joint positions over time ---
        fig1, axes1 = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
        fig1.suptitle("Joint Positions Over Time", fontsize=14)

        times = np.array([t for t, _ in self.joint_positions])
        positions = np.stack([p for _, p in self.joint_positions])

        for i, ax in enumerate(axes1):
            ax.plot(times, positions[:, i], "b-", linewidth=0.8, label="actual")

            # Overlay predicted targets as scatter points at their inference times.
            if n_targets > 0:
                for t, tgt in self.predicted_targets:
                    t_points = t + np.arange(tgt.shape[0]) * 0.1  # waypoint_dt=0.1
                    ax.plot(t_points, tgt[:, i], "r.-", markersize=3, linewidth=0.5, alpha=0.7)

                # Add legend entry once.
                ax.plot([], [], "r.-", markersize=3, label="predicted target")

            ax.set_ylabel(f"{JOINT_NAMES[i]}\n(rad)", fontsize=8)
            ax.legend(loc="upper right", fontsize=7)
            ax.grid(True, alpha=0.3)

        axes1[-1].set_xlabel("Time (s)")
        fig1.tight_layout()

        # --- Figure 2: Raw action deltas per chunk ---
        if n_actions > 0:
            fig2, axes2 = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
            fig2.suptitle("Raw Action Deltas (per inference chunk)", fontsize=14)

            for i, ax in enumerate(axes2):
                for chunk_idx, (t, act) in enumerate(self.raw_actions):
                    step_times = t + np.arange(act.shape[0]) * 0.1
                    ax.plot(step_times, act[:, i], ".-", markersize=3, linewidth=0.8, alpha=0.7)

                ax.set_ylabel(f"{JOINT_NAMES[i]}\n(delta rad)", fontsize=8)
                ax.axhline(y=0, color="k", linewidth=0.5, linestyle="--")
                ax.grid(True, alpha=0.3)

            axes2[-1].set_xlabel("Time (s)")
            fig2.tight_layout()

        # --- Figure 3: Action magnitude summary ---
        if n_actions > 0:
            fig3, (ax_mag, ax_hist) = plt.subplots(1, 2, figsize=(12, 5))
            fig3.suptitle("Action Magnitude Analysis", fontsize=14)

            all_actions = np.concatenate([act for _, act in self.raw_actions], axis=0)

            # Per-joint boxplot
            ax_mag.boxplot(
                [all_actions[:, i] for i in range(6)],
                labels=JOINT_NAMES,
            )
            ax_mag.set_ylabel("Delta (rad)")
            ax_mag.set_title("Action Distribution per Joint")
            ax_mag.axhline(y=0, color="k", linewidth=0.5, linestyle="--")
            ax_mag.grid(True, alpha=0.3)

            # Overall magnitude histogram
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

            # Print summary stats.
            print("\n=== Action Summary ===")
            print(f"Total chunks: {n_actions}")
            print(f"Total action steps: {all_actions.shape[0]}")
            print(f"Action magnitude — median: {np.median(magnitudes):.4f}, "
                  f"mean: {np.mean(magnitudes):.4f}, max: {np.max(magnitudes):.4f}")
            print(f"Per-joint mean deltas: {np.mean(all_actions, axis=0)}")
            print(f"Per-joint std deltas:  {np.std(all_actions, axis=0)}")

            if np.median(magnitudes) < 1e-6:
                print("\n*** WARNING: Action magnitudes are near-zero. ***")
                print("*** The model is likely returning zeros (decode failures). ***")

        plt.show()


def main():
    rclpy.init()
    node = EvalRecorderNode()

    # In replay mode the plot is already shown; just clean up.
    if node.replay_path:
        node.destroy_node()
        rclpy.shutdown()
        return

    # Handle Ctrl+C gracefully — save bag, then plot.
    def signal_handler(sig, frame):
        node.get_logger().info("Interrupted. Saving bag and generating plots...")
        node._finalize()
        node.destroy_node()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
