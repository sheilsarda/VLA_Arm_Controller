# 1. Set the installation path
$env:isaac_sim_package_path = "C:\isaacsim"

# 2. Tell Isaac Sim to use ROS 2 Jazzy and FastDDS
$env:ROS_DISTRO = "jazzy"
$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# 3. Add the internal Jazzy libraries to your Windows PATH (run this only once per session)
$env:PATH = "$env:PATH;$env:isaac_sim_package_path\exts\isaacsim.ros2.bridge\jazzy\lib"

# 4. Launch Isaac Sim with the Bridge and Domain ID enabled
& "$env:isaac_sim_package_path\isaac-sim.bat" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge --/app/exts/isaacsim.ros2.bridge/domain_id=42