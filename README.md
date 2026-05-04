## ros2_hybrid_astar_nav2

Hybrid A* global path planner in **Python** for **ROS 2 Humble**, integrated with a **five-node delivery pipeline** (sensor processing, obstacle tracking + prediction, local supervisor, safety `cmd_vel` gate, and Nav2-compatible global planning actions).

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

### Five-node architecture

| # | Node executable | Role |
|---|-----------------|------|
| 1 | `sensor_processor_node` | Sanitizes `LaserScan` (e.g. `/scan` → `/scan_filtered`). |
| 2 | `obstacle_tracker_node` | Segments scans, associates tracks in `map`, estimates velocity, publishes `ObstacleTrackArray`. |
| 3 | `local_planner_node` | Publishes predicted obstacle `PointCloud2` (`tracked_obstacles_cloud`), path clearance, `nav_speed_scale`, `nav_emergency_stop`. |
| 4 | `ackermann_safety_controller_node` | Consumes Nav2 `cmd_vel_smoothed`, applies scale / e-stop, publishes final `cmd_vel` (same pattern as `megoldas_sim24` `Twist` control). |
| 5 | `nav2_hybrid_astar_server` | Nav2 actions `compute_path_to_pose` / `compute_path_through_poses` (replaces default `planner_server` actions). |

Optional: `hybrid_astar_planner_node` exposes `GetPlan` for debugging; Nav2-only setups can ignore it.

Launch the pipeline together:

```bash
ros2 launch hybrid_astar_planner delivery_stack.launch.py use_sim_time:=true
```

Run **Nav2 bringup** (e.g. `nav2_hybrid_astar_bringup.launch.py`) in another terminal so `map`, TF, and `cmd_vel_smoothed` exist. Remap or retune `path_topic` on `local_planner_node` if your stack publishes the global plan on a topic other than `planned_path` (for example Nav2’s `plan` when enabled).

### Dynamic obstacles in the costmap

Point the `obstacle_layer` (or `voxel_layer`) at `tracked_obstacles_cloud` so predicted positions are inflated. Example pattern (adjust to your `nav2_params` layout):

- Add an observation source for the point cloud topic `tracked_obstacles_cloud` in the **local** costmap (and optionally global).

### Gazebo Classic + `megoldas_sim24`

The previous project drives the robot with `geometry_msgs/Twist` on `cmd_vel`. The Nav2 launch in this repo publishes the velocity smoother on **`cmd_vel_smoothed`** so **`ackermann_safety_controller_node`** can subscribe there and publish the final **`cmd_vel`** to Gazebo / the driver. If you run Nav2 **without** the safety node, add a relay or remap `cmd_vel_smoothed` → `cmd_vel` yourself.

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

### Modern Gazebo corner-to-corner action test

To avoid Classic SDF compatibility issues with the `roboworks` model, use modern Gazebo:

```bash
ros2 launch hybrid_astar_planner gz_modern_nav2_corner_test.launch.py
```

Convenience scripts:

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/clean_start.sh
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/start_corner_test.sh
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh
```

From Windows, you can use:

```bat
\wsl.localhost\Ubuntu-22.04\home\dominik\ros2_ws\src\ros2_hybrid_astar_nav2\hybrid_astar_planner\scripts\start_corner_test.bat
\wsl.localhost\Ubuntu-22.04\home\dominik\ros2_ws\src\ros2_hybrid_astar_nav2\hybrid_astar_planner\scripts\send_corner_goal.bat
```

This launch:
- starts modern Gazebo on the lightweight gray map,
- spawns `roboworks` in one corner (`-8, -8` by default),
- brings up Nav2 + the Python Hybrid A* action planner,
- starts RViz debug GUI with LiDAR + path + obstacle overlays,
- keeps manual goal triggering (no auto-goal script yet).

RViz debug visualization contains:
- `/scan` (raw LiDAR),
- `/scan_filtered` (filtered LiDAR),
- `/planned_path` and `/plan`,
- `/tracked_obstacles_cloud`,
- TF + map overlays.

Send a corner goal with helper script:

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh
```

Optional custom goal:

```bash
bash ~/ros2_ws/src/ros2_hybrid_astar_nav2/hybrid_astar_planner/scripts/send_corner_goal.sh 8.0 8.0 0.0 1.0
```

You can still send a manual Nav2 goal directly:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 8.0, y: 8.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
```

`start_corner_test.sh` now writes run logs here:
- full stream: `~/ros2_ws/logs/nav2_runs/corner_test_<timestamp>.log`
- filtered warnings/errors: `~/ros2_ws/logs/nav2_runs/corner_test_<timestamp>_errors.log`

**Troubleshooting (WSL / sim):** If the robot in RViz appears to “phase through” walls or the pose jumps, check Gazebo odometry publish rate first. The `roboworks` model’s Ackermann plugin must publish odometry at a reasonable rate (for example 50 Hz); at 1 Hz, `odom` → `base_link` TF is too sparse, which breaks TF extrapolation, confuses AMCL, and makes the RViz footprint update in large steps so motion can look like clipping. The packaged `model.sdf` sets `odom_publish_frequency` accordingly.

### Status

Active development: full navigation stack integration, tuning, and costmap wiring are environment-specific.
