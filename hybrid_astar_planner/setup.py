from pathlib import Path

from setuptools import find_packages, setup

package_name = "hybrid_astar_planner"


def _collect_files(relative_root: str) -> list[str]:
    root = Path(relative_root)
    if not root.exists():
        return []
    return [str(p) for p in root.rglob("*") if p.is_file()]


def _collect_data_files(relative_root: str, install_root: str):
    """Yield setuptools data_files entries that preserve subdirectory layout.

    setuptools' ``data_files`` flattens file basenames into a single install
    directory. To install ``sim/worlds/foo.sdf`` at
    ``share/<pkg>/sim/worlds/foo.sdf`` (rather than ``share/<pkg>/sim/foo.sdf``)
    we emit one ``(install_dir, [files])`` tuple per source subdirectory.
    """
    root = Path(relative_root)
    if not root.exists():
        return []
    groups: dict[str, list[str]] = {}
    for p in root.rglob("*"):
        if not p.is_file():
            continue
        rel_parent = p.parent.relative_to(root).as_posix()
        if rel_parent in ("", "."):
            target_dir = install_root
        else:
            target_dir = f"{install_root}/{rel_parent}"
        groups.setdefault(target_dir, []).append(str(p))
    return [(install_dir, files) for install_dir, files in groups.items()]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/demo_planner.launch.py",
                "launch/gz_sim_roboworks.launch.py",
                "launch/nav2_hybrid_astar_bringup.launch.py",
                "launch/nav2_bringup_no_planner.launch.py",
                "launch/nav2_navigation_no_planner.launch.py",
                "launch/mock_tf_tree.launch.py",
                "launch/delivery_stack.launch.py",
                "launch/gazebo_classic_warehouse.launch.py",
                "launch/gz_modern_nav2_corner_test.launch.py",
                "launch/gz_modern_nav2_corner_test_withRPP.launch.py",
            ],
        ),
        ("share/" + package_name + "/config", ["config/nav2_roboworks_minimal.yaml"]),
        (
            "share/" + package_name + "/rviz",
            ["rviz/hybrid_astar_demo.rviz", "rviz/nav2_delivery_debug.rviz"],
        ),
        (
            "share/" + package_name + "/scripts",
            [
                "scripts/clean_start.sh",
                "scripts/start_corner_test.sh",
                "scripts/send_corner_goal.sh",
            ],
        ),
        ("share/" + package_name + "/maps", ["maps/empty_map.yaml", "maps/empty_map.pgm"]),
        (
            "share/" + package_name + "/maps",
            ["maps/warehouse_lightweight_map.yaml", "maps/warehouse_lightweight_map.pgm"],
        ),
    # Simulation assets (SDF worlds, models, meshes). _collect_data_files
    # preserves the subdirectory structure so that the launch files can rely
    # on canonical paths like share/<pkg>/sim/worlds/<world>.sdf and the
    # GZ_MODEL_PATH model:// resolver finds share/<pkg>/sim/models/<name>/model.sdf.
    *_collect_data_files("sim", "share/" + package_name + "/sim"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="you@example.com",
    description="Hybrid A* global planner in Python (ROS 2 Humble) with RViz2 visualization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hybrid_astar_planner_node = hybrid_astar_planner.planner_node:main",
            "nav2_hybrid_astar_server = hybrid_astar_planner.nav2_hybrid_astar_server:main",
            "sensor_processor_node = hybrid_astar_planner.sensor_processor_node:main",
            "obstacle_tracker_node = hybrid_astar_planner.obstacle_tracker_node:main",
            "local_planner_node = hybrid_astar_planner.local_planner_node:main",
            "ackermann_safety_controller_node = hybrid_astar_planner.ackermann_safety_controller_node:main",
            "odom_tf_bridge_node = hybrid_astar_planner.odom_tf_bridge_node:main",
            "initial_pose_seed_node = hybrid_astar_planner.initial_pose_seed_node:main",
            "global_costmap_compat_node = hybrid_astar_planner.global_costmap_compat_node:main",
            "motion_reason_listener = hybrid_astar_planner.motion_reason_listener:main",
            "tf_event_logger_node = hybrid_astar_planner.tf_event_logger_node:main",
            "anchor_frame_guard_node = hybrid_astar_planner.anchor_frame_guard_node:main",
            "reverse_recovery_node = hybrid_astar_planner.reverse_recovery_node:main",
            "replan_watchdog_node = hybrid_astar_planner.replan_watchdog_node:main",
        ],
    },
)

