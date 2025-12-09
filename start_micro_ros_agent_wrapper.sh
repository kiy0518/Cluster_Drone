#!/bin/bash
# Micro-ROS Agent wrapper script for systemd
# This script sets up the environment and starts the agent

export HOME=/home/khadas
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source Micro-ROS Agent workspace (필수!)
if [ -f /home/khadas/Cluster_Drone/micro_ros_ws/install/setup.bash ]; then
    source /home/khadas/Cluster_Drone/micro_ros_ws/install/setup.bash
fi

# Source PX4 ROS2 messages workspace
if [ -f /home/khadas/Cluster_Drone/px4_ros2_ws/install/setup.bash ]; then
    source /home/khadas/Cluster_Drone/px4_ros2_ws/install/setup.bash
fi

# Set library path for Micro-ROS Agent (모든 필요한 경로 포함)
export LD_LIBRARY_PATH=/home/khadas/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}

# Start Micro-ROS Agent directly (ros2 run 대신 직접 실행)
exec /home/khadas/Cluster_Drone/micro_ros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent udp4 --port 8888

