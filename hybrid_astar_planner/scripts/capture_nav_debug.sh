#!/usr/bin/env bash
set -eo pipefail

# Usage:
#   bash capture_nav_debug.sh [duration_seconds]
# Example:
#   bash capture_nav_debug.sh 240

DURATION="${1:-180}"
TS="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$HOME/ros2_ws/logs/nav_debug_${TS}"
mkdir -p "$OUT_DIR"

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "[capture_nav_debug] duration=${DURATION}s"
echo "[capture_nav_debug] output=${OUT_DIR}"

# Snapshot one-time diagnostics.
{
  echo "=== date ==="
  date
  echo "=== node list ==="
  ros2 node list || true
  echo "=== topic list ==="
  ros2 topic list || true
  echo "=== tf_static once ==="
  timeout 8 ros2 topic echo /tf_static --once || true
  echo "=== params ==="
  ros2 param get /amcl transform_tolerance || true
  ros2 param get /controller_server FollowPath.transform_tolerance || true
} > "${OUT_DIR}/snapshot.txt" 2>&1

# Background capture helper.
PIDS=()
start_capture() {
  local name="$1"
  local cmd="$2"
  timeout "${DURATION}" bash -lc "${cmd}" > "${OUT_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
}

# Topic rates.
start_capture "hz_topics" \
  "ros2 topic hz /tf /odom_combined /planned_path /scan_filtered /cmd_vel_safe"

# TF chain monitoring.
start_capture "tf_map_odom" "ros2 run tf2_ros tf2_echo map odom"
start_capture "tf_odom_base" "ros2 run tf2_ros tf2_echo odom base_link"
start_capture "tf_map_base" "ros2 run tf2_ros tf2_echo map base_link"
start_capture "tf_base_laser" "ros2 run tf2_ros tf2_echo base_link laser"

# Core topics.
start_capture "planned_path" "ros2 topic echo /planned_path"
start_capture "odom_combined" "ros2 topic echo /odom_combined"
start_capture "scan_filtered" "ros2 topic echo /scan_filtered"
start_capture "cmd_vel_nav" "ros2 topic echo /cmd_vel_nav"
start_capture "cmd_vel_safe" "ros2 topic echo /cmd_vel_safe"
start_capture "amcl_pose" "ros2 topic echo /amcl_pose"

# Action state.
start_capture "nav_feedback" "ros2 topic echo /navigate_to_pose/_action/feedback"
start_capture "nav_status" "ros2 topic echo /navigate_to_pose/_action/status"

# Optional custom listener if present.
start_capture "motion_reason_listener" "ros2 run hybrid_astar_planner motion_reason_listener"

wait || true

echo "[capture_nav_debug] complete: ${OUT_DIR}"
