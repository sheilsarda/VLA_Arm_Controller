### Summary

This is Sheil's submission to Multiply Lab's technical challenge. Code is organized across 5 different modules:

#### Robot Controller (`robot_controller.py`)
Main orchestrator. Loads station poses from `grex_location_data.csv` and rail positions from `module_data.csv`. Plans trajectories via `PathPlanner`, checks them against AABB workspace obstacles, then executes via waypoint streaming. Monitors for collisions/faults during motion and has a basic force-vector backoff recovery stub.

Entry point: `python robot_controller.py` - loops through all stations continuously.

#### Robot Initializer (`robot_initializer.py`)
Dashboard server client (port 29999). Handles power-on, brake release, and spawns a background thread polling `robotmode`/`safetystatus`/`running` for fault detection.

This has been tested in isolation, and a demo video of the initialization sequence working end-to-end in UR Sim is present in the demo videos folder!

#### Collision Detection Agent (`collision_detection_agent.py`)
50Hz monitoring thread. Checks TCP force magnitude, joint currents, joint temps, velocities, and unexpected resistance (high force + low velocity). Reports `AnomalyEvent` objects; auto-triggers safety stop on critical anomalies. Configurable thresholds via `MonitoringThresholds`.

The anomaly events are monitored within `robot_controller.py`, and the planner is set up to halt any active goals the robot is working on when those occur.

#### Path Planner (`path_planner.py`)
- IKPy for inverse kinematics (URDF: `ur5e_collision.urdf`)
    - This is derived from the original URDF (universalUR5e.urdf) which contains mesh file references, except we needed to convert the meshs to primitives like cylinders to use it as a standalone file
- Linear interpolation in joint space with a configurable number of waypoints
- PyBullet for collision checking against axis-aligned bounding box obstacles transformed to robot base frame

#### Robot Communication (`robot_comms_for_ur_sim.py` and `robot_communication.py`)
- `robot_comms_for_ur_sim.py`: Real interface. Singleton `URRobotState` connects to real-time port 30003 (125Hz state stream) and primary port 30001 (URScript commands). Parses packet offsets for joint positions/velocities/currents, TCP pose/force, temps.
- `robot_communication.py`: Stubbed functions returning dummy values for offline testing.

Toggle via import in `robot_controller.py` and `collision_detection_agent.py`.

### Config Files
- `grex_location_data.csv`: Station name, XYZ position (mm), euler angles (deg)
- `module_data.csv`: Station name, rail position (mm), module width (mm)

### Dependencies
```
pip install -r requirements.txt
```
Core: `ikpy`, `pybullet`, `numpy`

### Known Issues / TODOs
- Trajectory to "Output for Scientist" sometimes reports false positive collision with "Add Media" AABB
- Joint tracking sometimes times out with 0.1-π/2 rad residual error
- Recovery flow incomplete (needs dashboard popup clear, protective stop unlock)
- No payload/gripper collision geometry in PyBullet yet
