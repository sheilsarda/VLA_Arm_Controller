import ikpy

def plan_trajectory(pose_start, pose_end, n_waypoints=50):
    """Generate collision-checkable trajectory."""
    
    chain = ikpy.chain.Chain.from_urdf_file("universalUR5e.urdf")
    # Get IK solutions at endpoints
    q_start_options = chain.inverse_kinematics(pose_start)
    q_end_options = chain.inverse_kinematics(pose_end)
    
    # Pick closest/smoothest solution pair (avoid big jumps)
    q_start, q_end = ikpy.utils.closest_pair(q_start_options, q_end_options)
    
    # Linear interpolation in joint space
    trajectory = []
    for i in range(n_waypoints):
        alpha = i / (n_waypoints - 1)
        q = (1 - alpha) * q_start + alpha * q_end
        trajectory.append(q)
    
    return trajectory