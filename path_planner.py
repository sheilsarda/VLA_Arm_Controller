from typing import List, Dict, Tuple, Optional
import pybullet as p
import pybullet_data
from ikpy.chain import Chain

class PathPlanner:
    def __init__(self):
        self.chain = Chain.from_urdf_file("universalUR5e.urdf")
        
        # PyBullet setup
        self._physics_client: Optional[int] = None
        self._robot_id: Optional[int] = None
        self._obstacle_bodies: Dict[str, int] = {}  # obstacle_name -> pybullet body ID
        self._joint_indices: List[int] = []  # Indices of movable joints in PyBullet
        
        self._init_pybullet()

    def _init_pybullet(self) -> None:
        """Initialize PyBullet physics server and load robot URDF."""
        # Use DIRECT mode for headless collision checking (no GUI overhead)
        # Switch to p.GUI for debugging/visualization
        self._physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self._robot_id = p.loadURDF(
            "universalUR5e.urdf",
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )
        
        # Cache movable joint indices (skip fixed joints)
        self._joint_indices = []
        for i in range(p.getNumJoints(self._robot_id)):
            joint_info = p.getJointInfo(self._robot_id, i)
            joint_type = joint_info[2]
            if joint_type != p.JOINT_FIXED:
                self._joint_indices.append(i)
        
        # Enable self-collision detection
        p.setCollisionFilterPair(
            self._robot_id, self._robot_id, -1, -1, enableCollision=True
        )

    def _create_obstacle_box(
        self,
        name: str,
        position: List[float],
        dimensions: Tuple[float, float, float]
    ) -> int:
        """
        Create an Axis Aligned Bounding Box obstacle in PyBullet for grex object
        
        Args:
            name: Unique identifier for this obstacle
            position: Center position [x, y, z] in robot base frame (meters)
            dimensions: (width_x, depth_y, height_z) in meters
            
        Returns:
            PyBullet body ID
        """
        half_extents = [d / 2.0 for d in dimensions]
        
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=half_extents
        )
        
        # Create as static body (mass=0)
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape,
            basePosition=position,
            baseOrientation=[0, 0, 0, 1]  # No rotation (axis-aligned)
        )
        
        self._obstacle_bodies[name] = body_id
        return body_id

    def _clear_obstacles(self) -> None:
        """Remove all obstacle bodies from simulation."""
        for body_id in self._obstacle_bodies.values():
            p.removeBody(body_id)
        self._obstacle_bodies.clear()

    def _transform_obstacle_to_robot_frame(
        self,
        obstacle_global_pos: List[float],
        rail_position: float
    ) -> List[float]:
        """
        Transform obstacle position from global frame to robot base_link frame.
        The robot base moves along X with the rail. Global 0 is at rail center.
        
        Args:
            obstacle_global_pos: [x, y, z] in global/world frame (meters)
            rail_position: Current rail position in meters (robot base X offset from global 0)
            
        Returns:
            [x, y, z] in robot base_link frame
        """
        return [
            obstacle_global_pos[0] - rail_position,  # Shift X by rail offset
            obstacle_global_pos[1],                   # Y unchanged
            obstacle_global_pos[2]                    # Z unchanged
        ]

    def _set_robot_configuration(self, joint_positions: List[float]) -> None:
        """
        Set robot joint positions in PyBullet.
        
        Args:
            joint_positions: List of joint angles (radians) for each movable joint
        """
        for idx, joint_idx in enumerate(self._joint_indices):
            if idx < len(joint_positions):
                p.resetJointState(self._robot_id, joint_idx, joint_positions[idx])

    def _check_collision_at_config(self) -> Tuple[bool, List[str], bool]:
        """
        Check for collisions at current robot configuration.
        
        Returns:
            Tuple of:
                - has_any_collision: bool
                - colliding_obstacles: List of obstacle names in collision
                - has_self_collision: bool
        """
        p.performCollisionDetection()
        
        colliding_obstacles = []
        has_self_collision = False
        
        # Check collisions with each obstacle
        for obstacle_name, obstacle_id in self._obstacle_bodies.items():
            contact_points = p.getContactPoints(
                bodyA=self._robot_id,
                bodyB=obstacle_id
            )
            if len(contact_points) > 0:
                colliding_obstacles.append(obstacle_name)
        
        # Check self-collision (robot vs itself)
        self_contacts = p.getContactPoints(
            bodyA=self._robot_id,
            bodyB=self._robot_id
        )
        # Filter out adjacent link contacts (these are expected at joints)
        for contact in self_contacts:
            link_a = contact[3]
            link_b = contact[4]
            # Adjacent links (diff of 1) are connected by joints - not real collisions
            if abs(link_a - link_b) > 1:
                has_self_collision = True
                break
        
        has_any_collision = len(colliding_obstacles) > 0 or has_self_collision
        return has_any_collision, colliding_obstacles, has_self_collision

    def plan_trajectory(self, start_position, target_position, target_orientation, n_waypoints=50):
        """Generate collision-checkable trajectory."""

        q_end = self.chain.inverse_kinematics(target_position, target_orientation) # returns list of the positions of each joint according to the target, including inactive joints
        
        # Linear interpolation in joint space
        trajectory = []
        for i in range(n_waypoints):
            alpha = i / (n_waypoints - 1)
            q_waypoint = [(1 - alpha) * start_position[joint] + alpha * q_end[joint] for joint in range(len(start_position))]
            trajectory.append(q_waypoint)
        return trajectory

    def check_trajectory_against_workspace_obstacles(
        self,
        trajectory: List[List[float]],
        start_rail_position: float,
        obstacles: List[Dict]
    ) -> List[str]:
        """
        Check if trajectory collides with workspace obstacles using PyBullet simulation.
        
        Args:
            trajectory: List of joint configurations (each is list of 6 joint angles in radians)
            start_position: Initial joint configuration (unused but kept for API compat)
            start_rail_position: Current rail position in mm (robot base X offset)
            obstacles: List of obstacle dicts with keys:
                - 'name': Obstacle identifier string
                - 'position': [x, y, z] in global frame (mm)
                - 'module_width': width along X (mm)
                - 'depth_from_rail': depth along Y (mm)
                - 'height': height along Z (mm)
                
        Returns:
            List of obstacle names that collide with the trajectory.
            Includes "SELF_COLLISION" if self-collision detected.
            
        TODO: Add payload collision checking once gripper + GRex geometry is defined.
        """

        self._clear_obstacles()
        
        # Convert units: input is mm, PyBullet expects meters
        MM_TO_M = 0.001
        rail_pos_m = start_rail_position * MM_TO_M
        
        for i, obs in enumerate(obstacles):
            obs_name = obs.get('name', f'obstacle_{i}')
            
            # Transform obstacle center from global frame to robot base frame
            global_pos = [coord * MM_TO_M for coord in obs['position']]
            local_pos = self._transform_obstacle_to_robot_frame(global_pos, rail_pos_m)
            
            dimensions = (
                obs['module_width'] * MM_TO_M,      # X (width along rail)
                obs['depth_from_rail'] * MM_TO_M,   # Y (depth from rail)
                obs['height'] * MM_TO_M             # Z (height)
            )
            
            self._create_obstacle_box(obs_name, local_pos, dimensions)
        
        # Check each waypoint in trajectory
        colliding_obstacles = set()
        has_self_collision = False
        
        for joint_config in trajectory:
            self._set_robot_configuration(joint_config)            
            _, obs_collisions, self_collision = self._check_collision_at_config()
            
            colliding_obstacles.update(obs_collisions)
            if self_collision:
                has_self_collision = True
        
        # Build result list
        result = list(colliding_obstacles)
        if has_self_collision:
            result.append("SELF_COLLISION")
        
        return result

    def __del__(self):
        """Cleanup PyBullet on destruction."""
        if self._physics_client is not None:
            try:
                p.disconnect(self._physics_client)
            except p.error:
                pass  # Already disconnected or invalid
