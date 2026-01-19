from ikpy.chain import Chain

class PathPlanner:
    def __init__(self):
        self.chain = Chain.from_urdf_file("universalUR5e.urdf")

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
    
    def check_trajectory_against_workspace_obstacles(self, trajectory, start_position, start_rail_position, obstacles):
        """
        TODO: Use Pybullet or some other sim to check if the trajectory shows robot is in collision with any workspace obstacles.
        """