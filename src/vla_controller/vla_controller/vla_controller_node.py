import threading
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from openpi_client import image_tools
from openpi_client import websocket_client_policy
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class VLAControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_controller_node")

        # openpi connection
        self.declare_parameter("openpi_host", "localhost")
        self.declare_parameter("openpi_port", 8000)

        # topics
        self.declare_parameter("base_camera_topic", "/camera/image_raw")
        self.declare_parameter("wrist_camera_topic", "/camera_wrist/image_raw")
        self.declare_parameter("joint_state_topic", "/joint_states")

        # action server
        self.declare_parameter(
            "arm_action_name", "/ur_manipulator_controller/follow_joint_trajectory"
        )

        # model/request behavior
        self.declare_parameter("task_instruction", "pick up the block")
        self.declare_parameter("action_mode", "velocity")  # velocity|delta
        self.declare_parameter("action_scale", 1.0)
        self.declare_parameter("velocity_scale", 1.0)
        self.declare_parameter("open_loop_horizon", 5)
        self.declare_parameter("waypoint_dt", 0.1)
        self.declare_parameter("dry_run", False)

        # joint config
        self.declare_parameter(
            "joint_names",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.declare_parameter("gripper_joint", "robotiq_85_left_knuckle_joint")
        self.declare_parameter("joint_lower_limits", [-6.283185] * 6)
        self.declare_parameter("joint_upper_limits", [6.283185] * 6)

        # read params
        self.openpi_host = str(self.get_parameter("openpi_host").value)
        self.openpi_port = int(self.get_parameter("openpi_port").value)

        self.base_camera_topic = str(self.get_parameter("base_camera_topic").value)
        self.wrist_camera_topic = str(self.get_parameter("wrist_camera_topic").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)

        self.arm_action_name = str(self.get_parameter("arm_action_name").value)
        self.task_instruction = str(self.get_parameter("task_instruction").value)
        self.action_mode = str(self.get_parameter("action_mode").value)
        self.action_scale = float(self.get_parameter("action_scale").value)
        self.velocity_scale = float(self.get_parameter("velocity_scale").value)
        self.open_loop_horizon = int(self.get_parameter("open_loop_horizon").value)
        self.waypoint_dt = float(self.get_parameter("waypoint_dt").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.gripper_joint = str(self.get_parameter("gripper_joint").value)
        self.joint_lower_limits = np.asarray(
            list(self.get_parameter("joint_lower_limits").value), dtype=np.float64
        )
        self.joint_upper_limits = np.asarray(
            list(self.get_parameter("joint_upper_limits").value), dtype=np.float64
        )

        if len(self.joint_names) != 6:
            raise ValueError("joint_names must contain exactly 6 joints for UR5e")
        if self.joint_lower_limits.shape[0] != 6 or self.joint_upper_limits.shape[0] != 6:
            raise ValueError("joint_lower_limits and joint_upper_limits must each have length 6")
        if self.action_mode not in {"velocity", "delta"}:
            raise ValueError("action_mode must be one of: velocity, delta")

        # ROS interfaces
        self.bridge = CvBridge()
        self.create_subscription(Image, self.base_camera_topic, self._base_image_cb, 10)
        self.create_subscription(Image, self.wrist_camera_topic, self._wrist_image_cb, 10)
        self.create_subscription(JointState, self.joint_state_topic, self._joint_state_cb, 30)
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)

        # cached state
        self._latest_base_img: Optional[np.ndarray] = None
        self._latest_wrist_img: Optional[np.ndarray] = None
        self._joint_positions: Dict[str, float] = {}
        self._lock = threading.Lock()

        # policy client handled from background thread because constructor blocks until server is reachable
        self._policy_client: Optional[websocket_client_policy.WebsocketClientPolicy] = None
        self._connect_thread = threading.Thread(target=self._connect_policy_client, daemon=True)
        self._connect_thread.start()

        self._next_infer_time = time.monotonic()
        self._inflight_goal = False
        self._timer = self.create_timer(0.05, self._control_tick)

        self.get_logger().info(
            f"vla_controller_node started (schema=droid, action_mode={self.action_mode}, dry_run={self.dry_run})"
        )

    def _connect_policy_client(self) -> None:
        try:
            self.get_logger().info(
                f"Connecting to openpi server at ws://{self.openpi_host}:{self.openpi_port}"
            )
            client = websocket_client_policy.WebsocketClientPolicy(
                host=self.openpi_host, port=self.openpi_port
            )
            with self._lock:
                self._policy_client = client
            self.get_logger().info("Connected to openpi server")
        except Exception as exc:
            self.get_logger().error(f"Failed to connect to openpi server: {exc}")

    def _base_image_cb(self, msg: Image) -> None:
        converted = self._convert_image(msg)
        if converted is not None:
            with self._lock:
                self._latest_base_img = converted

    def _wrist_image_cb(self, msg: Image) -> None:
        converted = self._convert_image(msg)
        if converted is not None:
            with self._lock:
                self._latest_wrist_img = converted

    def _joint_state_cb(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = float(pos)

    def _convert_image(self, msg: Image) -> Optional[np.ndarray]:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception:
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            except Exception as exc:
                self.get_logger().warn(f"Image conversion failed: {exc}")
                return None

        resized = image_tools.resize_with_pad(img, 224, 224)
        return image_tools.convert_to_uint8(resized)

    def _get_joint_vector(self) -> Optional[np.ndarray]:
        with self._lock:
            try:
                values = [self._joint_positions[name] for name in self.joint_names]
                return np.asarray(values, dtype=np.float64)
            except KeyError:
                return None

    def _get_gripper_value(self) -> float:
        with self._lock:
            return float(self._joint_positions.get(self.gripper_joint, 0.0))

    def _build_observation(self, joints: np.ndarray, gripper: float) -> Optional[dict]:
        with self._lock:
            base = None if self._latest_base_img is None else self._latest_base_img.copy()
            wrist = None if self._latest_wrist_img is None else self._latest_wrist_img.copy()

        if base is None or wrist is None:
            return None

        # Keep the bridge on one schema for now to reduce integration complexity:
        # DROID-compatible keys for pre-trained pi0/pi0.5 DROID checkpoints.
        padded_joints = np.concatenate([joints.astype(np.float32), np.zeros(1, dtype=np.float32)])
        return {
            "observation/exterior_image_1_left": base,
            "observation/wrist_image_left": wrist,
            "observation/joint_position": padded_joints,
            "observation/gripper_position": np.asarray([gripper], dtype=np.float32),
            "prompt": self.task_instruction,
        }

    def _compute_joint_targets(self, current: np.ndarray, actions: np.ndarray, n_steps: int) -> List[np.ndarray]:
        joints = current.copy()
        targets: List[np.ndarray] = []

        for i in range(n_steps):
            a = actions[i, :6].astype(np.float64) * self.action_scale

            if self.action_mode == "velocity":
                joints = joints + a * self.waypoint_dt * self.velocity_scale
            else:
                joints = joints + a

            joints = np.clip(joints, self.joint_lower_limits, self.joint_upper_limits)
            targets.append(joints.copy())

        return targets

    def _send_trajectory(self, targets: List[np.ndarray]) -> None:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for i, q in enumerate(targets):
            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            t = self.waypoint_dt * float(i + 1)
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        if self.dry_run:
            self.get_logger().info(
                f"dry_run=true, not sending trajectory. Last target={np.array2string(targets[-1], precision=3)}"
            )
            return

        if not self.arm_action_client.server_is_ready():
            self.get_logger().warn(
                f"Action server {self.arm_action_name} not ready yet; dropping this chunk"
            )
            return

        self._inflight_goal = True
        send_future = self.arm_action_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future) -> None:
        self._inflight_goal = False
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Trajectory goal rejected by controller")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_cb)
        except Exception as exc:
            self.get_logger().warn(f"Goal response error: {exc}")

    def _goal_result_cb(self, future) -> None:
        try:
            result = future.result().result
            err = result.error_code
            if err != 0:
                self.get_logger().warn(f"Trajectory execution finished with error_code={err}")
        except Exception as exc:
            self.get_logger().warn(f"Goal result error: {exc}")

    def _control_tick(self) -> None:
        now = time.monotonic()
        if now < self._next_infer_time:
            return

        if self._inflight_goal:
            return

        with self._lock:
            client = self._policy_client

        if client is None:
            return

        joints = self._get_joint_vector()
        if joints is None:
            return

        gripper = self._get_gripper_value()
        obs = self._build_observation(joints, gripper)
        if obs is None:
            return

        try:
            result = client.infer(obs)
        except Exception as exc:
            self.get_logger().warn(f"openpi inference failed: {exc}")
            self._next_infer_time = now + 0.5
            return

        if "actions" not in result:
            self.get_logger().warn("openpi response missing 'actions' key")
            self._next_infer_time = now + 0.5
            return

        actions = np.asarray(result["actions"])
        if actions.ndim != 2 or actions.shape[1] < 6:
            self.get_logger().warn(f"Unexpected action shape: {actions.shape}")
            self._next_infer_time = now + 0.5
            return

        n_steps = min(self.open_loop_horizon, actions.shape[0])
        targets = self._compute_joint_targets(joints, actions, n_steps)
        self._send_trajectory(targets)

        self._next_infer_time = now + (n_steps * self.waypoint_dt)


def main() -> None:
    rclpy.init()
    node = VLAControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
