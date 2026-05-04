#!/usr/bin/env bash
set -eo pipefail

WS_ROOT="/home/dominik/ros2_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${WS_ROOT}/logs/nav2_runs"
mkdir -p "${LOG_DIR}"
RUN_TS="$(date +%Y%m%d_%H%M%S)"
FULL_LOG="${LOG_DIR}/corner_test_${RUN_TS}.log"
ERROR_LOG="${LOG_DIR}/corner_test_${RUN_TS}_errors.log"

"${SCRIPT_DIR}/clean_start.sh"

echo "[start_corner_test] Sourcing ROS 2 and workspace..."
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"

echo "[start_corner_test] Launching modern Gazebo + Nav2 + RViz..."
echo "[start_corner_test] Full log: ${FULL_LOG}"
echo "[start_corner_test] Error log: ${ERROR_LOG}"

ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test.launch.py 2>&1 \
  | tee "${FULL_LOG}" \
  | awk '/\[ERROR\]|\[WARN\]|Exception|Traceback|Failed|failed|aborted|ABORTED/ { print; fflush(); }' > "${ERROR_LOG}"

LAUNCH_EXIT="${PIPESTATUS[0]}"
echo "[start_corner_test] Launch exited with code: ${LAUNCH_EXIT}"
exit "${LAUNCH_EXIT}"
