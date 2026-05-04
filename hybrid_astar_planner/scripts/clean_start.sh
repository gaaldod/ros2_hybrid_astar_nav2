#!/usr/bin/env bash
set -euo pipefail

echo "[clean_start] Stopping stale ROS/Gazebo processes..."

pkill -f "ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test.launch.py" || true
pkill -f "ros2 action send_goal /navigate_to_pose" || true
pkill -f "hybrid_astar_planner/odom_tf_bridge_node" || true
pkill -f "hybrid_astar_planner/initial_pose_seed_node" || true
pkill -f "hybrid_astar_planner/nav2_hybrid_astar_server" || true
pkill -f "hybrid_astar_planner/sensor_processor_node" || true
pkill -f "hybrid_astar_planner/obstacle_tracker_node" || true
pkill -f "hybrid_astar_planner/local_planner_node" || true
pkill -f "hybrid_astar_planner/ackermann_safety_controller_node" || true
pkill -f "component_container_isolated" || true
pkill -f "parameter_bridge" || true
pkill -f "ros_gz_bridge" || true
pkill -f "ign gazebo" || true
pkill -f "gz sim" || true
pkill -f "ros_gz_sim" || true
pkill -f "rviz2" || true

sleep 1
echo "[clean_start] Done."
