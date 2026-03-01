# 1. Set the installation path
$env:isaac_sim_package_path = "C:\isaacsim"

# 2. Tell Isaac Sim to use ROS 2 Jazzy and FastDDS
$env:ROS_DISTRO = "jazzy"
$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# 3. Add the internal Jazzy libraries to your Windows PATH (run this only once per session)
$env:PATH = "$env:PATH;$env:isaac_sim_package_path\exts\isaacsim.ros2.bridge\jazzy\lib"

# 3b. Point FastDDS at the unicast peer config so it can discover the ROS2 Docker container.
#     Copy fastdds_windows_side.xml from the repo's setup_scripts/ to a local Windows path first.
$env:FASTRTPS_DEFAULT_PROFILES_FILE = "$PSScriptRoot\fastdds_windows_side.xml"

# 3c. Open FastDDS ports for domain 42 in Windows Firewall (run once as Administrator).
#     Port formula: 7400 + 250*domain_id + offset  =>  domain 42 base = 17900
# New-NetFirewallRule -DisplayName "ROS2 FastDDS domain 42" -Direction Inbound -Protocol UDP -LocalPort 17900-17930 -Action Allow

# 4. Launch Isaac Sim with the Bridge and Domain ID enabled
& "$env:isaac_sim_package_path\isaac-sim.bat" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge --/app/exts/isaacsim.ros2.bridge/domain_id=42