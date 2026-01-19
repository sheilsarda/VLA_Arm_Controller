### Summary

This is Sheil's submission to Multiply Lab's technical challenge. Code is organized across 5 different modules, and I was able to tackle all 5 questions posed in the Technical Challenge.

<video controls src="Demo videos from Sheil/Trajectory planner demo (4x speed).mp4" title="Title"></video>

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

- `robot_communication.py`: Unchanged from the as-provided version. My understanding is that these are stubbed functions returning dummy values for offline testing.
- `robot_comms_for_ur_sim.py`: Since the stubbed version above doesn't hook into UR Sim, and thus can't be used to end-to-end test motion planning, collision avoidance, etc. I went ahead and filled in the stubs by reading UR documentation. 
    - This has been tested end-to-end in UR Sim for the UR5e robot; I'm not sure which specific arm in the lineup to use here, so based on trial and error chose the UR5 since it was able to reach all the locations we need to plan between for this challenge (based on GREX location data)
- To toggle between the stubbed version and the implemented one, we need to toggle 2 import statements for which comms methods to use. These are present at the top of the files in `robot_controller.py` and `collision_detection_agent.py`.

### Config Files
- `grex_location_data.csv`: Station name, XYZ position (mm), euler angles (deg)
- `module_data.csv`: Station name, rail position (mm), module width (mm)

### Dependencies
```
pip install -r requirements.txt
```

To run the planner end to end with UR Sim, follow [these](https://www.universal-robots.com/download/software-e-series/simulator-non-linux/offline-simulator-e-series-ur-sim-for-non-linux-5126-lts/) instructions to download UR Sim. On my Windows machine, I ended up using VMWare Workstation.

### Instructions to run this once dependencies are installed

- Note: To run this in a URSim environment, first toggle the comms methods to pull from the `robot_comms_for_ur_sim` file instead of the stubbed one. Else, leave things as-is
- `python robot_controller.py`
    - By default, the main method here will infinitely loop through the exercise of starting from any state, enabling the robot arm for movement, planning sequentially through all the targets and executing those targets

    - In the background, we have a couple of things runnning for safety / collision avoidance:
        - continous monitoring of safety status of the UR arm
        - continous monitoring of speeds, currents, etc. to detect anomalies