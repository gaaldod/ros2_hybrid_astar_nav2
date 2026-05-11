## ros2_hybrid_astar_nav2

Hybrid A* global planner and Nav2 delivery-stack integration for **ROS 2 Humble**. The ROS package is `hybrid_astar_planner`; the repository directory is `ros2_hybrid_astar_nav2`.

GitHub: <https://github.com/gaaldod/ros2_hybrid_astar_nav2>

## Overview

This project runs a Python Hybrid A* global planner with Nav2, Regulated Pure Pursuit (RPP), a Gazebo Fortress warehouse simulation, obstacle tracking, safety velocity gating, recovery/watchdog nodes, TF/AMCL diagnostics, and anchor-based relocalization.

The current primary simulation path is Gazebo Fortress plus:

```bash
ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test_withRPP.launch.py
```

## Dependencies

Install ROS 2 Humble first, then add the main runtime packages:

```bash
sudo apt update
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-tf-transformations
```

Notes:

- `delivery_robot_msgs` is an in-workspace package dependency, not an apt package. Keep it in the same `~/ros2_ws/src` workspace and build from the workspace root.
- Gazebo Classic packages such as `ros-humble-gazebo-ros-pkgs` and `ros-humble-gazebo-plugins` are only needed for the legacy Classic/AWS material below.

## Setup and Build

Clone this repository into a ROS workspace:

```bash
cd ~/ros2_ws/src
git clone -b main https://github.com/gaaldod/ros2_hybrid_astar_nav2.git ros2_hybrid_astar_nav2
```

Build the package and its workspace dependencies:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to hybrid_astar_planner
source install/setup.bash
```

## Run Current Simulation

From a sourced workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test_withRPP.launch.py
```

This launch is the current end-to-end entry point. It brings up the Gazebo Fortress warehouse, roboworks model, Nav2/RPP stack, Hybrid A* planner, delivery stack, bridge configuration, RViz, and moving obstacle driver.

## Send a Goal

Default corner goal, normally `(8, 8)` in `map`:

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh
```

Custom script goal arguments are `x y yaw_z yaw_w`:

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh 5.0 -2.0 0.7071 0.7071
```

Direct Nav2 action goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 8.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

The `goal_pose` topic is also used by recovery/watchdog/anchor nodes for cached goal republish:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 4.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

## Current Pipeline

Nav2 controller output is remapped to `cmd_vel_nav`. The safety controller gates it and publishes `cmd_vel_safe` to Gazebo through the bridge.

| Node | Executable | Role |
|------|------------|------|
| Sensor | `sensor_processor_node` | `/scan` to `/scan_filtered` |
| Perception | `obstacle_tracker_node` | Tracks obstacles in `map`, publishes `tracked_obstacles` and `tracked_obstacles_cloud` |
| Supervisor | `local_planner_node` | Obstacle cloud, speed scaling, emergency stop |
| Safety | `ackermann_safety_controller_node` | `cmd_vel_nav` to `cmd_vel_safe`, recovery arbitration, path rejoin, front collision stop |
| Global planner | `nav2_hybrid_astar_server` | Nav2 planning actions, live overlay, local takeover, localization jump guard |
| Recovery | `reverse_recovery_node` | Reverse / stall / collision-hint recovery |
| Watchdog | `replan_watchdog_node` | Optional periodic / stall replan hooks |
| Diagnostics | `tf_event_logger_node` | TF/clock/AMCL jump logging |
| Relocalization | `anchor_frame_guard_node` | Drift check, `/initialpose` reseed, persistent drift stop, goal republish |
| Gazebo obstacle | `moving_obstacle_driver_node` | Drives the physical `moving_box` model through bridged Gazebo topics |
| Nav2 compat | `global_costmap_compat_node` | No-op service for BT expectations |

Emergency stop inputs are ORed by the safety path:

| Topic | Source |
|-------|--------|
| `nav_emergency_stop` | `local_planner_node` |
| `nav_localization_jump_stop` | `nav2_hybrid_astar_server` |
| `nav_anchor_drift_stop` | `anchor_frame_guard_node` |

## Dynamic Obstacle

The current Gazebo Fortress world embeds a `moving_box` in `hybrid_astar_planner/sim/worlds/warehouse_lightweight_gz.sdf`. The box uses Gazebo `VelocityControl` and `OdometryPublisher` plugins.

`hybrid_astar_planner/sim/bridge_minimal.yaml` bridges:

- ROS `/moving_box/cmd_vel` to Gazebo `/model/moving_box/cmd_vel`
- Gazebo `/model/moving_box/odometry` to ROS `/moving_box/odometry`

`moving_obstacle_driver_node` publishes a constant `/moving_box/cmd_vel` and reverses direction from `/moving_box/odometry` near the configured waypoints. This node drives the physical Gazebo box only; it is not the older fake obstacle publisher `moving_obstacles_node`.

The lidar sees the box through its collision geometry. For Nav2 costmap integration, point the obstacle or voxel layer at `tracked_obstacles_cloud` in the local costmap, and optionally in the global costmap.

## Diagnostics

Useful live checks:

```bash
ros2 topic echo /nav_localization_jump_stop
ros2 topic echo /nav_anchor_drift_stop
ros2 topic echo /cmd_vel_safe
ros2 topic echo /moving_box/odometry
tail -f ~/ros2_ws/logs/nav2_runs/tf_event_*.log
```

Anchor guard current launch overrides in `delivery_stack.launch.py`:

- `persistent_drift_stop_yaw_deg=15.0`
- `persistent_drift_stop_dist_m=0.6`
- `persistent_drift_stop_sec=8.0`
- `reseed_cooldown_sec=10.0`

When the persistent drift threshold is sustained, `anchor_frame_guard_node` asserts `/nav_anchor_drift_stop`, attempts `/initialpose` reseed, and can republish the cached `/goal_pose` after reseed.

## Legacy / Optional

Gazebo Classic and AWS warehouse assets are kept as optional legacy material. They are useful for comparison and older integration tests, but they are not the recommended current path.

Classic warehouse launch:

```bash
ros2 launch hybrid_astar_planner gazebo_classic_warehouse.launch.py
```

Optional full AWS assets:

```bash
ros2 launch hybrid_astar_planner gazebo_classic_warehouse.launch.py use_full_aws_assets:=true
```

If you run Nav2 without the delivery safety node, add your own relay/remap from the controller output to the robot driver topic. In the current delivery stack, `ackermann_safety_controller_node` is responsible for publishing the final safe command stream.

AWS warehouse attribution:

- <https://github.com/aws-robotics/aws-robomaker-small-warehouse-world>

## Troubleshooting / Status

If RViz appears to show the robot phasing through walls or jumping, check Gazebo odometry and TF first. Sparse odometry can break TF extrapolation, confuse AMCL, and make the footprint update in large visual steps.

For localization discontinuities, correlate `tf_event_*.log`, `/nav_localization_jump_stop`, `/nav_anchor_drift_stop`, planner warnings in `/rosout`, and AMCL output.

Current source-of-truth files:

- `hybrid_astar_planner/launch/gz_modern_nav2_corner_test_withRPP.launch.py`
- `hybrid_astar_planner/launch/delivery_stack.launch.py`
- `hybrid_astar_planner/launch/gz_sim_roboworks.launch.py`
- `hybrid_astar_planner/sim/bridge_minimal.yaml`
- `hybrid_astar_planner/sim/worlds/warehouse_lightweight_gz.sdf`

The stack is still under active tuning. Planner, safety, anchor, bridge, and costmap behavior should be verified against the launch files above before changing runtime assumptions.
