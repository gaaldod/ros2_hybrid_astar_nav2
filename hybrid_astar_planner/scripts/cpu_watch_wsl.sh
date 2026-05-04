#!/usr/bin/env bash
#
# Temporary CPU sampler for ROS 2 / Gazebo sessions under WSL2.
#
# Windows Task Manager shows high "VMMEM" because that is the WSL virtual machine;
# this script logs *which Linux processes* dominate CPU so you can tune or renice.
#
# Usage:
#   chmod +x cpu_watch_wsl.sh
#   ./cpu_watch_wsl.sh [logfile] [interval_seconds]
#
# Example:
#   ./cpu_watch_wsl.sh ~/ros2_ws/logs/cpu_watch_$(date +%Y%m%d_%H%M%S).log 3
#
set -uo pipefail

LOG="${1:-${TMPDIR:-/tmp}/ros_cpu_watch.log}"
INTERVAL="${2:-5}"

mkdir -p "$(dirname "$LOG")" 2>/dev/null || true

echo "Logging to: $LOG"
echo "Interval: ${INTERVAL}s — Ctrl+C to stop"
{
  echo "===== ros_cpu_watch_wsl started $(date -Iseconds) ====="
  echo "interval_s=${INTERVAL}"
  uname -a 2>/dev/null || true
  echo ""
} | tee -a "$LOG"

while true; do
  {
    echo "======== $(date -Iseconds) ========"
    echo "--- load average ---"
    uptime 2>/dev/null || true
    echo ""
    echo "--- top processes by %CPU (ps aux, CPU% = average since process start) ---"
    ps aux --sort=-%cpu 2>/dev/null | head -n 35
    echo ""
    echo "--- top batch (often closer to 'right now') ---"
    if top -b -n1 -o %CPU 2>/dev/null | head -n 40; then
      :
    elif top -b -n1 2>/dev/null | head -n 40; then
      :
    else
      echo "(top not available)"
    fi
    echo ""
    echo "--- likely ROS / sim stack (name match) ---"
    ps aux 2>/dev/null | grep -E '[r]os2|component_container|[g]z |[g]azebo|ignition|[p]ython3? |[n]av2|[r]viz2|gz_[a-z]+|[p]lanner|AMCL|amcl' || echo "(no matching lines)"
    echo ""
    if command -v pidstat >/dev/null 2>&1; then
      echo "--- pidstat (1s) top lines ---"
      pidstat -u 1 1 2>/dev/null | tail -n +3 || true
      echo ""
    fi
    echo ""
  } >> "$LOG" 2>&1
  sleep "${INTERVAL}"
done
