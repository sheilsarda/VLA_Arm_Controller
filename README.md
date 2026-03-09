### Summary

#### From a fresh ROS2 install, need the following dependencies

````sh
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-xacro -y
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers -y
````

#### Clone submodule for `ur_description` by running this from root directory

````sh
git submodule update --init
````

#### Fix rviz qt segfault issue preventing moveit assistant from working

````sh
wget http://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
sudo dpkg -i ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb 
source install/setup.bash
````

#### Then Run Moveit setup assistant

````sh
ros2 run moveit_setup_assistant moveit_setup_assistant
colcon build
source install/setup.bash
````

#### Then to run moveit demo

````sh
ros2 launch ur5e_isaac_moveit_config controller_v1.launch.py
````

##### To verify if Isaac sim is up and running, and controllers are as well, here are some diagnostic commands that speak for themself

````sh
   71  ros2 topic list
   72  ros2 control list_controllers
   87  ros2 topic echo /robot_description --once
   88  ros2 param get /robot_state_publisher use_sim_time
   89  ros2 param get /controller_manager use_sim_time
   92  ros2 topic info /robot_description --verbose
  107  ros2 topic echo /clock
  108  ros2 node info /controller_manager
  109  ros2 control list_hardware_interfaces
````

#### VLA Controller (OpenPI + ROS2 Bridge)

Use two terminals.

Terminal 1: start the OpenPI inference server from the `openpi` repo:

````sh
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py --env=DROID
````

Equivalent explicit command:

````sh
cd /home/sheil/Development/openpi
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=pi05_droid \
  --policy.dir=gs://openpi-assets/checkpoints/pi05_droid
````

Terminal 2: build and launch the ROS2 side (controller manager + spawners + VLA bridge, no `move_group`):

````sh
cd /home/sheil/Development/VLA_Arm_Controller
source /opt/ros/jazzy/setup.bash

# Required once in the same Python environment used by ros2 launch:
pip install /home/sheil/Development/openpi/packages/openpi-client

colcon build
source install/setup.bash

ros2 launch vla_controller vla_system.launch.py \
  task:="pick up the block" \
  openpi_host:=localhost \
  openpi_port:=8000
````

If you want to launch only the bridge node (assuming controllers are already running elsewhere):

````sh
ros2 launch vla_controller vla.launch.py \
  task:="pick up the block" \
  openpi_host:=localhost \
  openpi_port:=8000
````

Quick checks before running the bridge:

````sh
ros2 topic list | grep camera
ros2 topic echo /joint_states --once
````
