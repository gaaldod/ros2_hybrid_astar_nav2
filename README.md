## ros2_hybrid_astar_nav2

Hybrid A* global path planner in **Python** for **ROS 2 Humble**, integrated with a **delivery pipeline** (sensor processing, obstacle tracking, local supervisor, safety `cmd_vel` gate, recovery/watchdog, TF/AMCL diagnostics, anchor-based relocalization) and Nav2-compatible global planning actions.

The repository is **self-contained** for version control.

### Dependencies

- **ROS 2 Humble** (`rclpy`, `nav2_*`, `tf2_*`, …)
- **Nav2** (for BT navigator + controller when you use the full stack):

  ```bash
  sudo apt update
  sudo apt install ros-humble-nav2-bringup
  ```

- **`delivery_robot_msgs`** (in this workspace) — custom obstacle track messages. Build the workspace from `ros2_ws` so both packages are picked up:

  ```bash
  cd ~/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --packages-select delivery_robot_msgs hybrid_astar_planner --symlink-install
  source install/setup.bash
  ```

### Command and safety pipeline (current default)

Nav2 is remapped so controller output is **`/cmd_vel_nav`** (see `nav2_navigation_no_planner.launch.py`). The delivery stack then:

1. **`local_planner_node`** — publishes `nav_speed_scale`, `nav_emergency_stop`, obstacle cloud on `tracked_obstacles_cloud`.
2. **`ackermann_safety_controller_node`** — subscribes to **`cmd_vel_nav`**, applies scale/limits/path assist/rejoin damping/front laser stop, and publishes **`cmd_vel_safe`** to the sim bridge.

**Emergency stop inputs (OR logic — any `true` zeros `cmd_vel_safe`):**

| Topic | Type | Source |
|-------|------|--------|
| `nav_emergency_stop` | `std_msgs/Bool` | `local_planner_node` |
| `nav_localization_jump_stop` | `std_msgs/Bool` | `nav2_hybrid_astar_server` during localization jump guard |
| `nav_anchor_drift_stop` | `std_msgs/Bool` | `anchor_frame_guard_node` during sustained anchor drift |

Launch the pipeline (optionally without the global planner if you start it elsewhere):

```bash
ros2 launch hybrid_astar_planner delivery_stack.launch.py use_sim_time:=true
```

### Runtime nodes (corner test + delivery)

| Node | Executable | Role |
|------|------------|------|
| Sensor | `sensor_processor_node` | `/scan` → `/scan_filtered` |
| Perception | `obstacle_tracker_node` | Tracks in `map`, publishes `tracked_obstacles` + cloud |
| Supervisor | `local_planner_node` | Obstacle cloud, speed scale, emergency stop |
| Safety | `ackermann_safety_controller_node` | `cmd_vel_nav` → `cmd_vel_safe`, dual aux stops, path rejoin, front collision stop |
| Global planner | `nav2_hybrid_astar_server` | Actions `compute_path_to_pose` / `compute_path_through_poses`, wall memory, live overlay, local takeover, jump guard + jump stop |
| Recovery | `reverse_recovery_node` | Reverse / stall / collision-hint recovery |
| Watchdog | `replan_watchdog_node` | Optional periodic / stall replan hooks |
| Diagnostics | `tf_event_logger_node` | TF/clock/AMCL jump log file + `/rosout` context |
| Relocalization | `anchor_frame_guard_node` | Drift vs odom anchor, `/initialpose` reseed, persistent drift stop, goal republish |
| Nav2 compat | `global_costmap_compat_node` | No-op service for BT expectations |

Optional: `hybrid_astar_planner_node` exposes `GetPlan` for debugging; Nav2-only setups can ignore it.

### Modern Gazebo corner test (`gz_modern_nav2_corner_test.launch.py`)

Primary sim + Nav2 + custom stack entry point:

```bash
ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test.launch.py
```

This launch (see source for exact defaults):

- Spawns **roboworks** in a corner (`spawn_x` / `spawn_y`, default **-8**, **-8**).
- Brings up **Nav2** with `nav2_bringup_no_planner` and a **RewrittenYaml** overlay (`_nav2_param_rewrites` in the launch file), including:
  - **AMCL**: `set_initial_pose` true, spawn-matched initial pose, tuned `update_min_a` / `update_min_d`, particle limits, `transform_tolerance`, `z_hit` / `z_rand` / `sigma_hit`, `laser_likelihood_max_dist`, `max_beams`.
  - **Controller (RPP)**: frequency, desired linear velocity, lookahead bands, Ackermann-oriented scaling, `transform_tolerance`.
  - **Costmaps**: footprint polygon, inflation, update frequencies.
  - **Velocity smoother** and **behavior_server** cycle frequency.
- Starts **`nav2_hybrid_astar_server`** with inline parameters (current file), e.g.:
  - `minimum_turning_radius: 1.35`, `allow_reverse: true`, `angle_quantization_bins: 72`
  - `planner_max_iterations: 12000`, `planner_timeout_sec: 20.0`
  - `wall_memory_inflation_radius_m: 0.9`
  - Penalties: `reverse_penalty`, `non_straight_penalty`, `direction_change_penalty`, `steering_change_penalty` as set in the launch file.
- Includes **`delivery_stack`** with `launch_hybrid_global:=false` so the Hybrid A* server is **not** duplicated (the corner launch owns the planner node).
- **TimerAction** delayed **initial_pose_seed_node** and **RViz**.
- **Static warehouse**: `sim/worlds/warehouse_lightweight_gz.sdf` (Fortress) and `sim/worlds/warehouse_lightweight.world` (Classic) define a 22 × 22 m enclosure with four perimeter walls and two interior shelf rows. Both files now live inside the `hybrid_astar_planner` package (`hybrid_astar_planner/sim/worlds/...`) and are installed automatically by `setup.py`, so the lidar in either Gazebo backend has real surfaces to scan and AMCL can localise against the matching occupancy grid.
- **Dynamic obstacle**: a `moving_box` model is embedded directly in `sim/worlds/warehouse_lightweight_gz.sdf` and driven by the Ignition Fortress `TrajectoryFollower` system plugin, so no ROS-side moving-obstacle node is required. The lidar sees it via the model's `<collision>` link.

**Hybrid A* tuning** lives mainly in `gz_modern_nav2_corner_test.launch.py` (planner node) and `delivery_stack.launch.py` (pipeline + anchor + safety). Edit those files rather than hunting scattered defaults.

### Custom goal examples

Default corner goal **(8, 8)** in `map` (same as `send_corner_goal.sh` defaults):

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh
```

Custom position and quaternion **z/w** (script args: `x y yaw_z yaw_w`):

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh 5.0 -2.0 0.7071 0.7071
```

Nav2 **NavigateToPose** action (full pose in `map`):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 8.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

**`goal_pose` topic** (used by `anchor_frame_guard_node`, `reverse_recovery_node`, `replan_watchdog_node` — republish / cache for recovery):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 3.0, y: 4.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}"
```

### Active listeners and diagnostics (what to `echo` / watch)

These nodes subscribe to live topics; use them for debugging TF jumps, stuck recovery, and false motion during bad localization.

| Node | Notable subscriptions (defaults) | Publishes / side effects |
|------|----------------------------------|---------------------------|
| **`tf_event_logger_node`** | `/clock`, `/amcl_pose`, `/odom_combined`, **`/cmd_vel_safe`**, `/rosout`; TF: `map`→`odom`, `odom`→`base_link` via listener | Rotating log: `~/ros2_ws/logs/nav2_runs/tf_event_<timestamp>.log` (`AMCL_JUMP`, `TF_JUMP`, context) |
| **`anchor_frame_guard_node`** | `/odom_combined`, `/amcl_pose`, **`/goal_pose`** | `/initialpose`, **`/nav_anchor_drift_stop`**, optional **`/goal_pose` republish** after reseed |
| **`nav2_hybrid_astar_server`** | `/rosout`, `/amcl_pose`, live obstacle cloud, map; TF for robot pose | **`/nav_localization_jump_stop`**, `planned_path`, planner actions |
| **`reverse_recovery_node`** | `planned_path`, `odom_combined`, **`cmd_vel_nav`**, **`cmd_vel_safe`**, **`goal_pose`**, `/rosout` | `nav_recovery_active`, `cmd_vel_recovery` |
| **`replan_watchdog_node`** | **`goal_pose`**, `cmd_vel_safe`, `planned_path` | Replans when enabled |
| **`ackermann_safety_controller_node`** | `cmd_vel_nav`, `nav_speed_scale`, `nav_emergency_stop`, **`nav_localization_jump_stop`**, **`nav_anchor_drift_stop`**, recovery topics, **`planned_path`**, **`odom_combined`**, **`scan_filtered`** | **`cmd_vel_safe`** |

Quick checks:

```bash
ros2 topic echo /nav_localization_jump_stop
ros2 topic echo /nav_anchor_drift_stop
ros2 topic echo /cmd_vel_safe
tail -f ~/ros2_ws/logs/nav2_runs/tf_event_*.log
```

### Convenience scripts

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/clean_start.sh
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/start_corner_test.sh
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh
```

`send_corner_goal.sh` captures **`/rosout`** to `~/ros2_ws/logs/nav2_runs/` and filters WARN+ on exit.

### Anchor guard: persistent drift stop (latest behavior)

When **`anchor_frame_guard`** sees anchor drift yaw above **`persistent_drift_stop_yaw_deg`** (default **5°**) continuously for **`persistent_drift_stop_sec`** (default **5 s**), it asserts **`/nav_anchor_drift_stop`** so the robot holds still, then attempts **`/initialpose`** reseed. On success it can **republish the last `/goal_pose`** after **`goal_republish_delay_sec`**. Blocked reseeds log explicit reasons (cooldown, speed gate). Tune in `delivery_stack.launch.py` under `anchor_frame_guard_node`.

### Dynamic obstacles in the costmap

Point the `obstacle_layer` (or `voxel_layer`) at `tracked_obstacles_cloud` so predicted positions are inflated. Example pattern (adjust to your `nav2_params` layout):

- Add an observation source for the point cloud topic `tracked_obstacles_cloud` in the **local** costmap (and optionally global).

### Gazebo Classic + `megoldas_sim24`

The previous project drives the robot with `geometry_msgs/Twist` on `cmd_vel`. This repo’s Nav2 launch remaps controller **`cmd_vel` → `cmd_vel_nav`** so **`ackermann_safety_controller_node`** can gate and publish **`cmd_vel_safe`** to Gazebo / the driver. If you run Nav2 **without** the safety node, add a relay or remap `cmd_vel_nav` → `cmd_vel` yourself.

If your bridge expects another topic (e.g. `roboworks/cmd_vel`), set `cmd_vel_out_topic` on `ackermann_safety_controller_node`.

### Gazebo Classic warehouse pilot map

For the first integration phase, this repository includes a tweaked Classic world:

- `sim/worlds/aws_small_warehouse_dynamic.world`
- 3 world-side scripted moving actors (for obstacle tracking / prediction tests)
- launch file: `gazebo_classic_warehouse.launch.py` (spawns `roboworks` into the map)

Run:

```bash
ros2 launch hybrid_astar_planner gazebo_classic_warehouse.launch.py
```

By default this launch uses a lightweight world (`warehouse_lightweight.world`) with
plain gray primitives and 3 moving cubes for lower CPU/RAM usage.

You can still switch back to the full AWS-asset world with:

```bash
ros2 launch hybrid_astar_planner gazebo_classic_warehouse.launch.py use_full_aws_assets:=true
```

The AWS model repository is cloned in this workspace as:

- `~/ros2_ws/src/aws-robomaker-small-warehouse-world`

Attribution (world and warehouse assets source):

- [aws-robotics/aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)

### Troubleshooting (WSL / sim)

If the robot in RViz appears to “phase through” walls or the pose jumps, check Gazebo odometry publish rate first. The `roboworks` model’s Ackermann plugin must publish odometry at a reasonable rate (for example 50 Hz); at 1 Hz, `odom` → `base_link` TF is too sparse, which breaks TF extrapolation, confuses AMCL, and makes the RViz footprint update in large steps so motion can look like clipping. The packaged `model.sdf` sets `odom_publish_frequency` accordingly.

For localization discontinuities, correlate **`tf_event_*.log`**, **`/nav_localization_jump_stop`**, and **`/nav_anchor_drift_stop`** with planner warnings in `/rosout`.

### Status

Active development: full navigation stack integration, tuning, and costmap wiring remain environment-specific. **Source of truth for “latest run” parameters** is the launch Python under `hybrid_astar_planner/launch/` (especially `gz_modern_nav2_corner_test.launch.py` and `delivery_stack.launch.py`).
