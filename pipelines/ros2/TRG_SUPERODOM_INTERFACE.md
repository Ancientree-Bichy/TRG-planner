# TRG-planner ROS 2 Humble Interface for SuperOdom

This ROS 2 node consumes SuperOdom localization and publishes a global TRG path.

## Build

Install the C++ core first:

```bash
cd /home/bichy/workspace/TRG-planner
sudo make deps
sudo make cppinstall
```

Build the ROS 2 package:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths /home/bichy/workspace/TRG-planner/pipelines/ros2
source install/setup.bash
```

## Run with SuperOdom

Start SuperOdom first. Its default `PROJECT_NAME` is empty, so the optimized
localization output is:

```text
/laser_odometry  nav_msgs/msg/Odometry
```

Then launch TRG-planner:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch trg_planner_ros trg_planner_wMap.py \
  map:=superodom_k_rail \
  params:=superodom_params.yaml \
  rviz:=true
```

Send goals with RViz `2D Goal Pose`, which publishes:

```text
/goal_pose  geometry_msgs/msg/PoseStamped
```

## Topics

Inputs:

| Topic | Type | Source | Purpose |
| --- | --- | --- | --- |
| `/laser_odometry` | `nav_msgs/msg/Odometry` | SuperOdom `laser_mapping_node` | Map-frame optimized pose used by TRG-planner. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz or autonomy layer | Global goal. Each new goal triggers planning immediately. |
| `/laser_cloud_map` | `sensor_msgs/msg/PointCloud2` | SuperOdom | Optional terrain cloud input when `map.isPrebuiltMap=false` or `trg.isUpdate=true`. |
| `/trg/input/pose` | `geometry_msgs/msg/PoseStamped` | Optional | Alternative pose input if odometry is not used. |

Outputs:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/trg/output/path` | `nav_msgs/msg/Path` | Smoothed global path for downstream tracking. |
| `/trg/output/prebuilt_map` | `sensor_msgs/msg/PointCloud2` | Loaded TRG prebuilt map for visualization. |
| `/trg/output/goal` | `sensor_msgs/msg/PointCloud2` | Current goal as a point cloud marker. |
| `/trg/debug/global_trg` | `visualization_msgs/msg/MarkerArray` | Global TRG nodes and edges. |
| `/trg/debug/local_trg` | `visualization_msgs/msg/MarkerArray` | Local TRG when online graph update is enabled. |
| `/trg/debug/obs_map` | `sensor_msgs/msg/PointCloud2` | Latest observation cloud accepted by TRG. |
| `/trg/debug/path_info` | `std_msgs/msg/Float32MultiArray` | `[direct_dist, raw_length, smooth_length, planning_time_ms, avg_risk]`. |

## Replanning

The node stores the latest goal and periodically checks the SuperOdom pose.
It requests a new global plan when either threshold is exceeded:

```yaml
ros2.replan.enabled: true
ros2.replan.checkRate: 2.0
ros2.replan.minInterval: 2.0
ros2.replan.minTranslation: 0.8
ros2.replan.minYaw: 0.35
```

This means TRG-planner checks odometry at 2 Hz and replans to the same goal
when the robot has moved at least 0.8 m or yaw has changed at least 0.35 rad
since the previous plan request, with at least 2 seconds between requests.

## Map Configuration

`config/superodom_k_rail.yaml` uses this static terrain map:

```text
/home/bichy/robocup_ws/SuperOdom/map_by_scanner/k-rail.pcd
```

That config now defaults to prior-map plus online-update mode (`trg.isUpdate: true`).
Change `map.prebuiltMapPath` to another absolute PCD path if needed. Absolute
paths and paths relative to the TRG-planner repository root are both supported.
