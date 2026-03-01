### Summary

TODO for ROS


#### From a fresh ROS2 install, need the following dependencies

````sh
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-xacro -y
````

Clone submodule for `ur_description` by running this from root directory

````sh
git submodule update --init
````

Fix rviz qt segfault issue preventing moveit assistant from working

````sh
wget http://snapshots.ros.org/jazzy/2025-05-23/ubuntu/pool/main/r/ros-jazzy-rviz-common/ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb
sudo dpkg -i ros-jazzy-rviz-common_14.1.11-1noble.20250520.201719_amd64.deb 
source install/setup.bash
````

Then Run setup assistant

````sh
ros2 run moveit_setup_assistant moveit_setup_assistant
````