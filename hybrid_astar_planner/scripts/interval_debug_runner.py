#!/usr/bin/env python3
from __future__ import annotations

import math
import re
import subprocess
import time
from pathlib import Path


def run(cmd: str, timeout: int = 30) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout)


def cleanup() -> None:
    cmds = [
        "bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/stop_corner_test.sh >/dev/null 2>&1 || true",
        "pkill -f 'ign gazebo' >/dev/null 2>&1 || true",
        "pkill -f rviz2 >/dev/null 2>&1 || true",
        "pkill -f 'motion_reason_listener' >/dev/null 2>&1 || true",
        "pkill -f 'send_corner_goal.sh' >/dev/null 2>&1 || true",
    ]
    for c in cmds:
        subprocess.run(["bash", "-lc", c], check=False)


def wait_action_server(timeout_s: int = 120) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        p = run("ros2 action info /navigate_to_pose", timeout=12)
        out = (p.stdout or "") + (p.stderr or "")
        if "Action servers: 1" in out:
            return True
        time.sleep(2)
    return False


def get_odom_xy() -> tuple[float, float] | None:
    p = run("timeout 8 ros2 topic echo /odom_combined --once", timeout=12)
    txt = (p.stdout or "") + (p.stderr or "")
    m = re.search(r"position:\n\s+x:\s*([-0-9.eE]+)\n\s+y:\s*([-0-9.eE]+)", txt)
    if not m:
        return None
    return float(m.group(1)), float(m.group(2))


def main() -> None:
    log_root = Path("/home/dominik/ros2_ws/logs/nav2_runs")
    log_root.mkdir(parents=True, exist_ok=True)
    summary = []

    for i in range(1, 4):
        cleanup()
        print(f"=== INTERVAL {i} START ===", flush=True)

        launch = subprocess.Popen(
            [
                "bash",
                "-lc",
                "ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test.launch.py launch_rviz:=false",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        ok = wait_action_server(120)
        if not ok:
            summary.append((i, "startup_fail"))
            launch.terminate()
            cleanup()
            continue

        listener_log = log_root / f"motion_reason_interval_{i}.log"
        listener = subprocess.Popen(
            [
                "bash",
                "-lc",
                f"ros2 run hybrid_astar_planner motion_reason_listener > {listener_log} 2>&1",
            ]
        )
        goal = subprocess.Popen(
            [
                "bash",
                "-lc",
                "bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        odom0 = get_odom_xy()
        t0 = time.time()
        samples = []
        while time.time() - t0 < 180:
            od = get_odom_xy()
            if od and odom0:
                samples.append(math.hypot(od[0] - odom0[0], od[1] - odom0[1]))
            time.sleep(20)

        odom1 = get_odom_xy()
        moved = None
        if odom0 and odom1:
            moved = math.hypot(odom1[0] - odom0[0], odom1[1] - odom0[1])
        max_moved = max(samples) if samples else None

        summary.append(
            (
                i,
                "ok",
                odom0,
                odom1,
                moved,
                max_moved,
                str(listener_log),
            )
        )
        print(f"=== INTERVAL {i} END moved={moved} max={max_moved} ===", flush=True)

        goal.terminate()
        listener.terminate()
        launch.terminate()
        cleanup()
        time.sleep(3)

    print("=== SUMMARY ===", flush=True)
    for row in summary:
        print(row, flush=True)


if __name__ == "__main__":
    main()
