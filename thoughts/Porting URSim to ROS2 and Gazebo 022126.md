# ROS Refactoring Plan for Robot Controller with URSim

## Executive Summary

This document outlines a plan to refactor the current robot controller from direct socket-based communication to ROS (Robot Operating System) while maintaining compatibility with URSim. This migration will provide better integration, standardization, and extensibility for the robotics system.

---

## Current Architecture

### Communication Layer
- **Direct TCP Socket Communication**
  - Port 30003: Real-time data streaming (125Hz) - robot state
  - Port 30001: Primary interface - URScript command execution
  - Port 29999: Dashboard server - robot initialization/status
  - Custom packet parsing for UR e-Series real-time interface
  - Manual URScript command generation

### Key Components
1. **`robot_comms_for_ur_sim.py`**: Low-level socket communication with URSim
2. **`robot_controller.py`**: High-level controller orchestrating path planning
3. **`robot_initializer.py`**: Robot power-on and brake release via dashboard server
4. **`path_planner.py`**: Trajectory planning logic
5. **`collision_detection_agent.py`**: Collision monitoring

---

## Benefits of ROS Migration

### 1. **Standardization**
   - Standard message types (`sensor_msgs/JointState`, `geometry_msgs/Pose`, etc.)
   - Consistent API across different robot platforms
   - Industry-standard communication patterns

### 2. **Robustness**
   - Built-in reconnection logic
   - Better error handling and recovery
   - Automatic node lifecycle management

### 3. **Integration & Extensibility**
   - Easy integration with visualization tools (RViz)
   - Compatibility with motion planning frameworks (MoveIt!)
   - Integration with other ROS packages (gazebo, tf2, etc.)
   - Better debugging with `rosbag` for data recording

### 4. **Maintainability**
   - Well-documented ROS packages
   - Active community support
   - Separation of concerns (nodes vs. topics)

### 5. **Simulation Support**
   - URSim can run ROS drivers
   - Better simulation-to-real transition
   - Standardized simulation interfaces

---

## Required ROS Packages

### Core ROS Packages
1. **`ur_robot_driver`** (or `ur_modern_driver` for older versions)
   - Official Universal Robots ROS driver
   - Handles real-time data streaming
   - Manages robot state and commands
   - Supports URSim via network connection

2. **`ur_msgs`**
   - Message definitions for UR robots
   - Joint states, robot modes, safety status

3. **`controller_manager`**
   - Manages robot controllers
   - Handles joint trajectory execution

4. **`joint_trajectory_controller`**
   - Executes joint space trajectories
   - Provides action interface for motion planning

### Optional but Recommended
5. **`moveit`** (if upgrading to MoveIt! for planning)
   - Advanced motion planning framework
   - Collision checking
   - Workspace visualization

6. **`tf2`** and **`tf2_ros`**
   - Transform management
   - Coordinate frame handling

---

## Proposed ROS Architecture

### Node Structure

```
┌─────────────────────────────────────────────────────────┐
│                    ROS Master                           │
└─────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ ur_robot_ │  │  robot_state  │  │ path_planner │
│   _driver    │  │     node      │  │     node     │
└──────────────┘  └──────────────┘  └──────────────┘
        │                 │                 │
        │                 │                 │
        └─────────────────┼─────────────────┘
                          │
                  ┌──────────────┐
                  │ robot_control│
                  │     node     │
                  └──────────────┘
```

### Topic Structure

**Published Topics:**
- `/joint_states` (sensor_msgs/JointState) - Current joint positions/velocities
- `/tf` (tf2_msgs/TFMessage) - Transform tree
- `/ur_hardware_interface/robot_program_running` (std_msgs/Bool)
- `/ur_hardware_interface/safety_mode` (ur_msgs/SafetyMode)
- `/ur_hardware_interface/robot_mode` (ur_msgs/RobotMode)

**Subscribed Topics:**
- `/scaled_joint_trajectory_controller/command` (trajectory_msgs/JointTrajectory) - Joint trajectory commands
- `/tool_controller/command` (std_msgs/Float64) - Tool control

**Services:**
- `/ur_hardware_interface/dashboard/...` - Dashboard commands (power on, brake release, etc.)

**Actions:**
- `/scaled_joint_trajectory_controller/follow_joint_trajectory` (control_msgs/FollowJointTrajectoryAction)

---

## Refactoring Strategy

### Phase 1: ROS Infrastructure Setup

#### 1.1 Create ROS Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

#### 1.2 Install UR ROS Packages
```bash
# Install from apt (ROS Noetic/Humble)
sudo apt-get install ros-<distro>-ur-robot-driver
sudo apt-get install ros-<distro>-ur-msgs

# Or clone from source
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
```

#### 1.3 Create Custom Package
```bash
cd ~/catkin_ws/src
catkin_create_pkg vla_arm_controller rospy std_msgs sensor_msgs geometry_msgs trajectory_msgs ur_msgs
```

### Phase 2: Communication Layer Refactoring

#### 2.1 Replace `robot_comms_for_ur_sim.py`

**Current:** Direct socket communication
```python
# Direct socket connection
_robot_state = URRobotState()
_robot_state.connect()
```

**New:** ROS topic subscribers/publishers
```python
# ROS-based communication
import rospy
from sensor_msgs.msg import JointState
from ur_msgs.msg import RobotMode, SafetyMode

class ROSRobotInterface:
    def __init__(self):
        rospy.init_node('robot_interface')
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_callback)
        self.robot_mode_sub = rospy.Subscriber('/ur_hardware_interface/robot_mode', RobotMode, self._robot_mode_callback)
        # ... other subscribers
```

#### 2.2 Refactor Functions

**Mapping of current functions to ROS:**

| Current Function | ROS Equivalent |
|----------------|----------------|
| `get_angular_pose()` | Subscribe to `/joint_states` → `JointState.position` |
| `get_base_cartesian_pose()` | Subscribe to `/tf` or compute from joint states |
| `move_to_angular_position()` | Publish to `/scaled_joint_trajectory_controller/command` |
| `move_to_base_cartesian_position()` | Use action client for `FollowJointTrajectoryAction` |
| `get_cartesian_base_force()` | Subscribe to `/wrench` or `/force_torque_sensor` |
| `get_tool_voltage()` | Subscribe to `/ur_hardware_interface/tool_data` |

### Phase 3: Robot Initialization Refactoring

#### 3.1 Replace `robot_initializer.py`

**Current:** Direct dashboard server socket
```python
s.connect((ROBOT_IP, DASHBOARD_SERVER_PORT))
s.sendall("power on\n".encode('utf-8'))
```

**New:** ROS service calls
```python
from ur_dashboard_msgs.srv import Load, IsProgramRunning, GetLoadedProgram

class ROSRobotInitializer:
    def __init__(self):
        rospy.wait_for_service('/ur_hardware_interface/dashboard/power_on')
        self.power_on_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/power_on', Trigger)
        self.brake_release_srv = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)
    
    def power_on_and_initialize_robot(self):
        response = self.power_on_srv()
        # ... handle response
```

### Phase 4: Controller Refactoring

#### 4.1 Refactor `robot_controller.py`

**Key Changes:**
1. Replace direct function calls with ROS topic/service calls
2. Use action clients for trajectory execution
3. Subscribe to robot state topics instead of polling
4. Use ROS parameters for configuration

**Example Refactoring:**

**Before:**
```python
current_joint_position = get_angular_pose()
move_to_angular_position(j1, j2, j3, j4, j5, j6)
```

**After:**
```python
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.joint_state = None
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_callback)
        self.trajectory_client = actionlib.SimpleActionClient(
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.trajectory_client.wait_for_server()
    
    def _joint_state_callback(self, msg):
        self.joint_state = msg
    
    def move_to_angular_position(self, j1, j2, j3, j4, j5, j6):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                                       'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        point = JointTrajectoryPoint()
        point.positions = [j1, j2, j3, j4, j5, j6]
        point.time_from_start = rospy.Duration(2.0)  # 2 seconds
        goal.trajectory.points = [point]
        
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()
```

### Phase 5: Configuration Files

#### 5.1 Launch Files
Create `launch/robot_control.launch`:
```xml
<launch>
    <!-- UR Robot Driver -->
    <include file="$(find ur_robot_driver)/launch/ur5e_bringup.launch">
        <arg name="robot_ip" value="192.168.40.128"/>
        <arg name="use_fake_hardware" value="false"/>
    </include>
    
    <!-- Robot Controller Node -->
    <node name="robot_controller" pkg="vla_arm_controller" type="robot_controller.py" output="screen">
        <rosparam file="$(find vla_arm_controller)/config/robot_params.yaml"/>
    </node>
</launch>
```

#### 5.2 Parameter Files
Create `config/robot_params.yaml`:
```yaml
robot_controller:
  default_height: 0.5
  default_depth_from_rail: 0.5
  robot_module_width_on_rail: 0.5
  grex_location_file: "$(find vla_arm_controller)/config/grex_location_data.csv"
  module_data_file: "$(find vla_arm_controller)/config/module_data.csv"
```

---

## Migration Steps (Detailed)

### Step 1: Environment Setup
1. Install ROS (Noetic for Ubuntu 20.04, or Humble for Ubuntu 22.04)
2. Install UR ROS packages
3. Set up catkin workspace
4. Test UR driver connection to URSim

### Step 2: Create ROS Package Structure
```
vla_arm_controller/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── scripts/
│   ├── robot_controller.py
│   ├── robot_interface.py
│   └── robot_initializer.py
├── launch/
│   ├── robot_control.launch
│   └── ur_sim.launch
├── config/
│   ├── robot_params.yaml
│   ├── grex_location_data.csv
│   └── module_data.csv
└── src/
    └── vla_arm_controller/
        ├── path_planner.py
        └── collision_detection_agent.py
```

### Step 3: Implement ROS Interface Layer
- Create `robot_interface.py` that wraps ROS topics/services
- Maintain same function signatures as current interface for minimal changes
- Implement as a compatibility layer initially

### Step 4: Gradual Migration
1. **Week 1:** Set up ROS infrastructure, test UR driver with URSim
2. **Week 2:** Implement ROS interface layer, test basic movement
3. **Week 3:** Migrate robot_initializer to ROS services
4. **Week 4:** Migrate robot_controller to use ROS interface
5. **Week 5:** Testing, debugging, and optimization

### Step 5: Testing Strategy
1. Unit tests for ROS interface functions
2. Integration tests with URSim
3. End-to-end trajectory execution tests
4. Performance comparison (latency, throughput)

---

## Code Structure Changes

### New File: `scripts/robot_interface.py`
```python
#!/usr/bin/env python3
"""
ROS-based robot interface maintaining compatibility with existing API.
"""
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Wrench
from ur_msgs.msg import RobotMode, SafetyMode
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from typing import List

class ROSRobotInterface:
    """ROS-based interface maintaining compatibility with existing function signatures."""
    
    def __init__(self):
        if not rospy.get_node_uri():
            rospy.init_node('robot_interface', anonymous=True)
        
        # State storage
        self.joint_state = None
        self.tcp_pose = None
        self.tcp_force = None
        self.robot_mode = None
        self.safety_mode = None
        
        # Subscribers
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_cb)
        # ... other subscribers
        
        # Action clients
        self.trajectory_client = actionlib.SimpleActionClient(
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.trajectory_client.wait_for_server(timeout=rospy.Duration(10.0))
    
    def _joint_state_cb(self, msg: JointState):
        self.joint_state = msg
    
    # Compatibility functions matching current API
    def get_angular_pose(self) -> List[float]:
        if self.joint_state is None:
            rospy.logwarn("No joint state received yet")
            return [0.0] * 6
        return list(self.joint_state.position)
    
    def move_to_angular_position(self, j1, j2, j3, j4, j5, j6):
        # Implementation using action client
        pass
    # ... other compatibility functions
```

### Modified: `scripts/robot_controller.py`
```python
#!/usr/bin/env python3
"""
Robot controller using ROS interface.
"""
import rospy
from robot_interface import ROSRobotInterface
# ... rest of imports

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        
        # Load parameters
        self.default_height = rospy.get_param('~default_height', 0.5)
        
        # Use ROS interface instead of direct socket
        self.robot_interface = ROSRobotInterface()
        
        # Rest of initialization...
    
    def get_angular_pose(self):
        return self.robot_interface.get_angular_pose()
    
    # ... rest of methods use robot_interface
```

---

## URSim Configuration for ROS

### URSim Setup
1. **Enable External Control in URSim:**
   - In URSim, go to Settings → Network
   - Enable "External Control" mode
   - Set IP address to match your ROS machine

2. **ROS Driver Configuration:**
   - Use `ur_robot_driver` with `robot_ip` parameter pointing to URSim
   - URSim typically runs on `127.0.0.1` or a specific IP

3. **Launch File for URSim:**
```xml
<launch>
    <arg name="robot_ip" default="127.0.0.1"/>
    <arg name="robot_type" default="ur5e"/>
    
    <include file="$(find ur_robot_driver)/launch/$(arg robot_type)_bringup.launch">
        <arg name="robot_ip" value="$(arg robot_ip)"/>
        <arg name="use_fake_hardware" value="false"/>
    </include>
</launch>
```

---

## Challenges & Solutions

### Challenge 1: Real-time Performance
**Issue:** ROS topics may have higher latency than direct socket communication.

**Solution:**
- Use ROS parameters to optimize topic queue sizes
- Consider using `rosbag` to analyze timing
- May need to tune UR driver parameters for lower latency

### Challenge 2: Coordinate Frame Management
**Issue:** Need to handle base frame, tool frame, and workspace frames.

**Solution:**
- Use `tf2` for coordinate frame transformations
- Publish transforms for rail position, tool frame, etc.
- Subscribe to transform tree for pose queries

### Challenge 3: Trajectory Execution Timing
**Issue:** Current code uses sleep() and polling; ROS uses callbacks.

**Solution:**
- Use action clients for trajectory execution (provides feedback)
- Implement state machine for trajectory execution
- Use ROS timers instead of sleep() where appropriate

### Challenge 4: Collision Detection Integration
**Issue:** `collision_detection_agent.py` may need ROS integration.

**Solution:**
- Publish collision status as ROS topic
- Subscribe to safety mode from UR driver
- Integrate with MoveIt! collision checking if using MoveIt!

---

## Testing Plan

### Unit Tests
- Test ROS interface functions return correct data types
- Test trajectory generation and execution
- Test error handling and reconnection

### Integration Tests
1. **Basic Movement Test:**
   - Connect to URSim via ROS
   - Execute simple joint trajectory
   - Verify robot moves correctly

2. **Full Workflow Test:**
   - Initialize robot
   - Plan trajectory to target
   - Execute trajectory
   - Verify completion

3. **Error Recovery Test:**
   - Simulate connection loss
   - Test reconnection logic
   - Test safety stop recovery

### Performance Tests
- Measure latency: command → execution start
- Measure trajectory execution time
- Compare with current socket-based implementation

---

## Rollback Plan

If issues arise during migration:

1. **Keep both implementations:**
   - Maintain `robot_comms_for_ur_sim.py` as fallback
   - Use feature flag to switch between ROS and socket-based

2. **Gradual rollout:**
   - Test ROS implementation thoroughly in simulation
   - Run both implementations in parallel initially
   - Switch over only after validation

3. **Version control:**
   - Create branch for ROS migration
   - Keep main branch stable
   - Merge only after full validation

---

## Timeline Estimate

| Phase | Duration | Description |
|-------|----------|-------------|
| Setup & Learning | 1 week | ROS installation, package setup, learning curve |
| Interface Layer | 1 week | Create ROS interface compatibility layer |
| Initialization | 3 days | Migrate robot_initializer to ROS services |
| Controller Migration | 1 week | Refactor robot_controller to use ROS |
| Testing & Debugging | 1 week | Integration testing, performance tuning |
| Documentation | 2 days | Update README, create launch file docs |
| **Total** | **~4-5 weeks** | Full migration with testing |

---

## Next Steps

1. **Decision Point:** Review this plan and decide on ROS distribution (Noetic vs. Humble)
2. **Environment Setup:** Install ROS and UR packages
3. **Proof of Concept:** Create minimal ROS node that connects to URSim
4. **Incremental Migration:** Start with interface layer, then migrate components one by one
5. **Validation:** Test each component before moving to next

---

## Additional Resources

- [Universal Robots ROS Driver Documentation](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [ROS UR Driver Tutorial](http://wiki.ros.org/universal_robots)
- [URSim Documentation](https://www.universal-robots.com/products/universal-robots-simulator/)
- [ROS Action Client Tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials)

---

## Questions to Consider

1. **ROS Distribution:** Which ROS version? (Noetic LTS vs. Humble)
2. **MoveIt! Integration:** Do we want to integrate MoveIt! for advanced planning?
3. **Real-time Requirements:** What are the latency requirements?
4. **Deployment:** Will this run on the same machine as URSim or separate?
5. **Hardware:** Will this eventually run on real hardware, or simulation only?

---

*This plan is a living document and should be updated as the migration progresses.*
