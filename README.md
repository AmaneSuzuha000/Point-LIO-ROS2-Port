# Point-LIO — ROS2 Humble Port

A personal porting project: **[Point-LIO](https://github.com/hku-mars/Point-LIO)** (ROS1)
migrated to **ROS2 Humble**, using `colcon` / `ament_cmake` / `rclcpp` and `tf2`.

- `point_lio_ros2/` — migrated package (colcon workspace, ament_cmake, rclcpp, tf2)
- `migration_guide.md` — step-by-step notes with real diffs, errors & solutions, build/run commands
- `evidence/` — build & run logs (`pl_build.log`: 0 error / 0 warning), 60s offline feed → 589 odometry messages at 9.558 Hz (`VERDICT PASS`), rviz2 render (`E_rviz2_pointlio.png`)
- `migration_lab/` — reproducible evidence capture + migrate/fix scripts
- `patch_*.py` — auditable structural rewrite rules (api / build / includes / launch / pcl_ptr / structural / time / yaml)
- `fast_lio_ros2/` + `fastlio_verify.md` — a second, optional migration example (FAST-LIO)

## Quick start

```bash
# Build (Ubuntu 22.04 + ROS2 Humble)
cd point_lio_ros2
colcon build --symlink-install

# Run with offline synthetic Livox data
source install/setup.bash
ros2 launch point_lio_ros2 mapping_avia.launch.py

# Reproduce all evidence
bash migration_lab/capture_evidence.sh
```

See `migration_guide.md` for the full porting details.
