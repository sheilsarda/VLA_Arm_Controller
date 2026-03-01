#!/bin/bash
# Source this file to configure ROS2 to discover Isaac Sim on the Windows host.
# Usage: source setup_scripts/ros2_isaac_env.sh

source /opt/ros/jazzy/setup.bash

# Match Isaac Sim's domain ID (set in windows-sim-startup.ps1)
export ROS_DOMAIN_ID=42

# Use FastDDS (matches Isaac Sim)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Point to the unicast peer discovery config
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fastdds_isaac_bridge.xml"

echo "[ros2_isaac_env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[ros2_isaac_env] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[ros2_isaac_env] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "[ros2_isaac_env] Ready. Run: ros2 topic list"
