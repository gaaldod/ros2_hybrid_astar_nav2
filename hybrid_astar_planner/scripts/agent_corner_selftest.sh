#!/usr/bin/env bash
set -e
WS="/home/dominik/ros2_ws"
SRC="${WS}/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts"
REPORT="${WS}/logs/nav2_runs/agent_selftest_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "${WS}/logs/nav2_runs"
exec > >(tee -a "$REPORT") 2>&1

echo "========== agent selftest start $(date -Iseconds) =========="
echo "Report: $REPORT"

source /opt/ros/humble/setup.bash
cd "$WS"
echo "[1/5] colcon build hybrid_astar_planner..."
colcon build --packages-select hybrid_astar_planner --symlink-install 2>&1 | tail -25
source install/setup.bash

echo "[2/5] starting corner test stack (background)..."
bash "${SRC}/start_corner_test.sh" &
LAUNCH_PID=$!
echo "start_corner_test PID=$LAUNCH_PID"

cleanup() {
  echo "[cleanup] stopping stack..."
  bash "${SRC}/stop_corner_test.sh" || true
}
trap cleanup EXIT

echo "[3/5] waiting for Gazebo + Nav2 (until /odom_combined appears, max ~3 min)..."
READY=0
for _ in $(seq 1 90); do
  if timeout 8 ros2 topic list 2>/dev/null | grep -q /odom_combined; then
    READY=1
    echo "Stack ready: /odom_combined found."
    break
  fi
  sleep 2
done
if [[ "$READY" -ne 1 ]]; then
  echo "WARN: /odom_combined not seen; continuing anyway."
fi
sleep 15

sample_topics () {
  local tag="$1"
  echo ""
  echo "----- TOPIC SAMPLE [$tag] $(date -Iseconds) -----"
  timeout 12 ros2 topic echo /odom_combined --once 2>&1 | head -45 || echo "(no odom)"
  timeout 8 ros2 topic echo /cmd_vel_safe --once 2>&1 | head -22 || echo "(no cmd_vel_safe)"
}

echo "[4/5] samples + goals (~10 min)"
sample_topics "T0"

echo "--- goal A (8,8) max 180s ---"
timeout 180 bash "${SRC}/send_corner_goal.sh" 2>&1 | tail -40 || true
sleep 60
sample_topics "T1"

sleep 90
sample_topics "T2_quiet"

echo "--- goal B (-8,-8) max 180s ---"
timeout 180 bash "${SRC}/send_corner_goal.sh" -8.0 -8.0 0.0 1.0 2>&1 | tail -40 || true
sleep 60
sample_topics "T3"

sleep 90
sample_topics "T4_quiet"

echo "--- goal C (8,8) max 180s ---"
timeout 180 bash "${SRC}/send_corner_goal.sh" 2>&1 | tail -40 || true
sleep 45
sample_topics "T5"

echo "[5/5] latest corner_test errors (tail) ---"
LATEST=$(ls -t "${WS}/logs/nav2_runs"/corner_test_*.log 2>/dev/null | head -1 || true)
if [[ -n "$LATEST" ]]; then
  grep -E "Failed to make progress|ABORTED|ERROR.*controller" "$LATEST" 2>/dev/null | tail -30 || true
fi

echo "========== agent selftest end $(date -Iseconds) =========="
