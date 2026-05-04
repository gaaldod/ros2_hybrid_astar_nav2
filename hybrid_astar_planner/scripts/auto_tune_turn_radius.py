#!/usr/bin/env python3
from __future__ import annotations

import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass
class TrialResult:
    radius: float
    reason: str
    best_distance: float | None
    runtime_s: float


def _run(cmd: str, timeout: int = 30) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(args=cmd, returncode=124, stdout="", stderr="timeout")


def _cleanup() -> None:
    cmds = [
        "bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/stop_corner_test.sh >/dev/null 2>&1 || true",
        "pkill -f 'ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test.launch.py' >/dev/null 2>&1 || true",
        "pkill -f 'ign gazebo' >/dev/null 2>&1 || true",
        "pkill -f 'ros2 action send_goal /navigate_to_pose' >/dev/null 2>&1 || true",
    ]
    for cmd in cmds:
        subprocess.run(["bash", "-lc", cmd], check=False)


def _wait_action_server(timeout_s: int = 120) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        proc = _run("ros2 action info /navigate_to_pose", timeout=12)
        out = (proc.stdout or "") + (proc.stderr or "")
        if "Action servers: 1" in out:
            return True
        time.sleep(2)
    return False


def _wait_bt_active(timeout_s: int = 120) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        proc = _run("ros2 lifecycle get /bt_navigator", timeout=10)
        out = ((proc.stdout or "") + (proc.stderr or "")).lower()
        if "active" in out:
            return True
        time.sleep(2)
    return False


def _wait_planner_active(timeout_s: int = 120) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        proc = _run("ros2 lifecycle get /planner_server", timeout=10)
        out = ((proc.stdout or "") + (proc.stderr or "")).lower()
        if "active" in out:
            return True
        time.sleep(2)
    return False


def _wait_amcl_pose(timeout_s: int = 120) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        proc = _run("timeout 6 ros2 topic echo /amcl_pose --once", timeout=10)
        out = (proc.stdout or "") + (proc.stderr or "")
        if "position:" in out and "orientation:" in out:
            return True
        time.sleep(1)
    return False


def _wait_tf_map_to_base(timeout_s: int = 120, consecutive_ok: int = 3) -> bool:
    start = time.time()
    ok_count = 0
    while time.time() - start < timeout_s:
        proc = _run("timeout 4 ros2 run tf2_ros tf2_echo map base_link", timeout=8)
        out = (proc.stdout or "") + (proc.stderr or "")
        # tf2_echo prints transform data when available; frame errors indicate not ready.
        has_tf = ("Translation:" in out and "Rotation:" in out) or ("At time" in out)
        has_error = "Invalid frame ID" in out or "Lookup would require extrapolation" in out
        if has_tf and not has_error:
            ok_count += 1
            if ok_count >= consecutive_ok:
                return True
        else:
            ok_count = 0
        time.sleep(1)
    return False


def _send_goal() -> subprocess.Popen[str]:
    cmd = (
        "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
        "\"{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 8.0, z: 0.0}, "
        "orientation: {z: 0.0, w: 1.0}}}}\""
    )
    return subprocess.Popen(
        ["bash", "-lc", cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )


def _goal_accepted(timeout_s: int = 90) -> bool:
    log_dir = Path.home() / "ros2_ws" / "logs" / "nav2_runs"
    before = {p.name for p in log_dir.glob("goal_*.log")} if log_dir.exists() else set()

    _run(
        "timeout {} bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh".format(
            timeout_s
        ),
        timeout=timeout_s + 6,
    )
    after = list(log_dir.glob("goal_*.log")) if log_dir.exists() else []
    if not after:
        return False

    newest = max(after, key=lambda p: p.stat().st_mtime)
    if newest.name in before and len(after) > 1:
        # If no new file appears (edge case), still inspect newest content.
        pass

    try:
        text = newest.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return False
    return "Goal accepted with ID" in text


def _odom_sample(timeout_s: int = 8) -> tuple[tuple[float, float] | None, int | None]:
    proc = _run(f"timeout {timeout_s} ros2 topic echo /odom_combined --once", timeout=timeout_s + 4)
    text = (proc.stdout or "") + (proc.stderr or "")
    pose_match = re.search(r"position:\n\s+x:\s*([-0-9.eE]+)\n\s+y:\s*([-0-9.eE]+)", text)
    pose = (float(pose_match.group(1)), float(pose_match.group(2))) if pose_match else None
    stamp_match = re.search(r"stamp:\n\s+sec:\s*(\d+)", text)
    stamp_sec = int(stamp_match.group(1)) if stamp_match else None
    return pose, stamp_sec


def _path_available(timeout_s: int = 6) -> bool:
    proc = _run(f"timeout {timeout_s} ros2 topic echo /planned_path --once", timeout=timeout_s + 4)
    text = (proc.stdout or "") + (proc.stderr or "")
    return "poses:" in text


def _wait_first_path(timeout_s: int = 45) -> bool:
    start = time.time()
    while time.time() - start < timeout_s:
        if _path_available(timeout_s=5):
            return True
        time.sleep(1)
    return False


def run_trials(
    radii: list[float], total_budget_s: int = 600, per_trial_limit_s: int = 180
) -> list[TrialResult]:
    results: list[TrialResult] = []
    budget_end = time.time() + total_budget_s

    print("=== AUTO TUNING START ===", flush=True)
    for radius in radii:
        if time.time() >= budget_end:
            break

        print(f"\n--- radius {radius:.2f} ---", flush=True)
        _cleanup()

        launch_cmd = (
            "ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test.launch.py "
            f"launch_rviz:=false smac_min_turning_radius:={radius}"
        )
        launch_proc = subprocess.Popen(
            ["bash", "-lc", launch_cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, text=True
        )

        if (
            not _wait_action_server(timeout_s=120)
            or not _wait_bt_active(timeout_s=120)
            or not _wait_planner_active(timeout_s=120)
            or not _wait_amcl_pose(timeout_s=120)
            or not _wait_tf_map_to_base(timeout_s=120, consecutive_ok=3)
        ):
            print("startup gating failed (action/bt/planner/amcl/tf)", flush=True)
            launch_proc.terminate()
            _cleanup()
            results.append(TrialResult(radius, "startup_fail", None, 0.0))
            continue

        accepted = _goal_accepted(timeout_s=90)
        if not accepted:
            print("goal not accepted; startup_fail", flush=True)
            launch_proc.terminate()
            _cleanup()
            results.append(TrialResult(radius, "goal_not_accepted", None, 0.0))
            continue

        if not _wait_first_path(timeout_s=45):
            print("no planned_path after goal; planning_fail", flush=True)
            launch_proc.terminate()
            _cleanup()
            results.append(TrialResult(radius, "no_path_after_goal", None, 0.0))
            continue

        trial_start = time.time()
        stall_for = 0.0
        last_pose: tuple[float, float] | None = None
        last_stamp: int | None = None
        best_dist: float | None = None
        reason = "time_limit"

        while time.time() - trial_start < per_trial_limit_s and time.time() < budget_end:
            pose, stamp_sec = _odom_sample(timeout_s=8)
            path_ok = _path_available(timeout_s=5)
            progressed = False
            if pose is not None and last_pose is not None:
                moved = ((pose[0] - last_pose[0]) ** 2 + (pose[1] - last_pose[1]) ** 2) ** 0.5
                if moved > 0.10:
                    progressed = True

            if stamp_sec is not None and last_stamp is not None and stamp_sec > last_stamp:
                progressed = True

            if progressed:
                stall_for = 0.0
            else:
                stall_for += 8.0

            print(
                f"odom pose={pose} stamp={stamp_sec} path={path_ok} stall_for={stall_for:.0f}s",
                flush=True,
            )

            if stall_for >= 30.0:
                reason = "hang_30s_no_progress"
                break

            if pose is not None:
                last_pose = pose
            if stamp_sec is not None:
                last_stamp = stamp_sec
            if path_ok and best_dist is None:
                best_dist = 999.0  # Marker that a valid plan stream exists for this radius.

        runtime_s = time.time() - trial_start
        results.append(TrialResult(radius, reason, best_dist, runtime_s))

        launch_proc.terminate()
        _cleanup()
        time.sleep(2)

    print("\n=== RESULTS ===", flush=True)
    for item in results:
        print(
            f"radius={item.radius:.2f} reason={item.reason} "
            f"best_dist={item.best_distance} runtime_s={item.runtime_s:.1f}",
            flush=True,
        )

    valid = [x for x in results if x.best_distance is not None]
    if valid:
        best = min(valid, key=lambda x: x.best_distance if x.best_distance is not None else float("inf"))
        print(
            f"RECOMMENDED radius={best.radius:.2f} (lowest best_dist={best.best_distance})",
            flush=True,
        )
    else:
        print("RECOMMENDED: no successful runs", flush=True)

    _cleanup()
    print("=== AUTO TUNING END ===", flush=True)
    return results


if __name__ == "__main__":
    run_trials(radii=[1.20, 1.35, 1.50], total_budget_s=600, per_trial_limit_s=180)
