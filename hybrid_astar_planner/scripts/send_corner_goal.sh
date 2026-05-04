#!/usr/bin/env bash
set -eo pipefail

WS_ROOT="/home/dominik/ros2_ws"
LOG_DIR="${WS_ROOT}/logs/nav2_runs"
mkdir -p "${LOG_DIR}"
RUN_TS="$(date +%Y%m%d_%H%M%S)"
GOAL_LOG="${LOG_DIR}/goal_${RUN_TS}.log"
WARN_ERR_LOG="${LOG_DIR}/warn_error_${RUN_TS}.log"
ROSOUT_RAW_LOG="${LOG_DIR}/rosout_raw_${RUN_TS}.log"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STOP_SCRIPT="${SCRIPT_DIR}/stop_corner_test.sh"

GOAL_X="${1:-8.0}"
GOAL_Y="${2:-8.0}"
GOAL_YAW_Z="${3:-0.0}"
GOAL_YAW_W="${4:-1.0}"

source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"

GOAL_PAYLOAD="{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {z: ${GOAL_YAW_Z}, w: ${GOAL_YAW_W}}}}}"

echo "[send_corner_goal] Goal target: (${GOAL_X}, ${GOAL_Y})"
echo "[send_corner_goal] Logging to: ${GOAL_LOG}"
echo "[send_corner_goal] WARN/ERROR log: ${WARN_ERR_LOG}"
echo "[send_corner_goal] Raw rosout log: ${ROSOUT_RAW_LOG}"

# Capture full rosout while the goal is active; filter WARN/ERROR/FATAL on shutdown.
ros2 topic echo /rosout > "${ROSOUT_RAW_LOG}" 2>/dev/null &
ROSOUT_CAPTURE_PID=$!

cleanup_capture() {
  if [[ -n "${ROSOUT_CAPTURE_PID:-}" ]] && kill -0 "${ROSOUT_CAPTURE_PID}" 2>/dev/null; then
    kill "${ROSOUT_CAPTURE_PID}" 2>/dev/null || true
    wait "${ROSOUT_CAPTURE_PID}" 2>/dev/null || true
  fi
  if [[ -f "${ROSOUT_RAW_LOG}" ]]; then
    awk 'BEGIN{RS="---"; ORS="---\n"}
      /(^|\n)level:[[:space:]]*(30|40|50)(\n|$)/ {print $0}
    ' "${ROSOUT_RAW_LOG}" > "${WARN_ERR_LOG}" || true
  fi
}

trap cleanup_capture EXIT

START_EPOCH="$(date +%s)"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "${GOAL_PAYLOAD}" 2>&1 | tee "${GOAL_LOG}"
SEND_EXIT="${PIPESTATUS[0]}"
END_EPOCH="$(date +%s)"
ELAPSED="$((END_EPOCH - START_EPOCH))"

if grep -q "status: 4" "${GOAL_LOG}" || grep -q "SUCCEEDED" "${GOAL_LOG}"; then
  RESULT_TEXT="SUCCEEDED"
elif grep -q "status: 6" "${GOAL_LOG}" || grep -q "ABORTED" "${GOAL_LOG}"; then
  RESULT_TEXT="ABORTED"
elif grep -q "status: 5" "${GOAL_LOG}" || grep -q "CANCELED" "${GOAL_LOG}"; then
  RESULT_TEXT="CANCELED"
else
  RESULT_TEXT="UNKNOWN"
fi

echo "[send_corner_goal] Result: ${RESULT_TEXT}"
echo "[send_corner_goal] Elapsed: ${ELAPSED}s"

if [[ "${SEND_EXIT}" -ne 0 ]]; then
  exit "${SEND_EXIT}"
fi

cleanup_capture
trap - EXIT

if [[ "${RESULT_TEXT}" == "ABORTED" ]]; then
  echo "[send_corner_goal] Goal aborted. Stopping running corner-test stack..."
  if [[ -x "${STOP_SCRIPT}" ]]; then
    "${STOP_SCRIPT}" || true
  else
    bash "${STOP_SCRIPT}" || true
  fi
  echo "[send_corner_goal] Stack shutdown finished. Logs saved."
fi
