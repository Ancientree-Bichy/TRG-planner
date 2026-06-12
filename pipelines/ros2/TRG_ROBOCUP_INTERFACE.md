# TRG-planner ROS 2 Humble Interface for RoboCup

This node consumes localization and publishes a global TRG path. The TRG
hyperparameters are scene-independent; the prior-map PCD path is supplied at
runtime.

## Build

```bash
cd $ROBOCUP_WS/TRG-planner
cmake -S cpp/trg_planner -B /tmp/trg_planner_core_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$ROBOCUP_WS/.robocup_install/trg_core
cmake --build /tmp/trg_planner_core_build --target install -j"$(nproc)"

source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=$ROBOCUP_WS/.robocup_install/trg_core:${CMAKE_PREFIX_PATH:-}
colcon build --base-paths pipelines/ros2 \
  --install-base $ROBOCUP_WS/.robocup_install/trg_ros2 \
  --build-base /tmp/trg_ros2_build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run TRG Only

```bash
source /opt/ros/humble/setup.bash
source $ROBOCUP_WS/.robocup_install/trg_ros2/setup.bash

ros2 launch trg_planner_ros trg_planner_wMap.py \
  config:=robocup_default \
  params:=robocup_params.yaml \
  map_path:=$ROBOCUP_WS/data/all_field_origin_down_0p5m.pcd \
  boundary_file:=$ROBOCUP_WS/bringup/config/boundaries/all_field_origin_down_0p5m_allowed_area.yaml \
  rviz:=true \
  rviz_config:=$ROBOCUP_WS/bringup/rviz/robocup_navigation.rviz
```

In RViz:

- `2D Pose Estimate` publishes `/initialpose`; the launch bridge converts it to
  `/laser_odometry` for TRG-only debugging.
- `2D Goal Pose` publishes `/goal_pose`.

## Topics

Inputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/laser_odometry` | `nav_msgs/msg/Odometry` | Map-frame localization pose used by TRG-planner. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Global goal. Each new goal triggers planning immediately. |
| `/laser_cloud_map` | `sensor_msgs/msg/PointCloud2` | Optional terrain cloud input when online update is enabled. |
| `/trg/input/pose` | `geometry_msgs/msg/PoseStamped` | Optional pose input if odometry is not used. |

Outputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/trg/output/path` | `nav_msgs/msg/Path` | Smoothed global path for downstream tracking. |
| `/trg/output/prebuilt_map` | `sensor_msgs/msg/PointCloud2` | Loaded prior map for visualization. |
| `/trg/output/goal` | `sensor_msgs/msg/PointCloud2` | Current goal marker. |
| `/trg/debug/global_trg` | `visualization_msgs/msg/MarkerArray` | Global TRG nodes and edges. |
| `/trg/debug/local_trg` | `visualization_msgs/msg/MarkerArray` | Local TRG when online graph update is enabled. |
| `/trg/debug/obs_map` | `sensor_msgs/msg/PointCloud2` | Latest observation cloud accepted by TRG. |
| `/trg/debug/path_info` | `std_msgs/msg/Float32MultiArray` | `[direct_dist, raw_length, smooth_length, planning_time_ms, avg_risk]`. |

## Configuration Rule

Use `config/robocup_default.yaml` as the only source hyperparameter YAML. Do not
create map-specific YAMLs. If a run needs a different prior map, pass
`map_path:=...` or let the RoboCup bringup script generate a runtime snapshot.

Path search mode is also a YAML concern. Keep `trg.pathSearchMode: native` for
the original TRG A* behavior, or set `trg.pathSearchMode: trg_astar` to use the
heading-aware C++ TRG-AStar search. The RoboCup bringup scripts expose the same
switch through `TRG_PATH_SEARCH_MODE`.
