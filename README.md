# LIO ROS Wrapper

ROS2 wrapper for LiDAR-Inertial Odometry system.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select lio_ros_wrapper
source install/setup.bash
```

## Usage

### Livox Mid360

```bash
ros2 launch lio_ros_wrapper lio_mid360.launch.py
```

Custom topics:
```bash
ros2 launch lio_ros_wrapper lio_mid360.launch.py \
  imu_topic:=/custom/imu \
  lidar_topic:=/custom/lidar
```

### Livox Avia

```bash
ros2 launch lio_ros_wrapper lio_avia.launch.py
```

## Topics

### Subscribed
- `/livox/mid360/imu` or `/livox/avia/imu` - IMU data (sensor_msgs/Imu)
- `/livox/mid360/lidar` or `/livox/avia/lidar` - LiDAR point cloud (sensor_msgs/PointCloud2)

### Published
- `/lio/odometry` - Odometry (nav_msgs/Odometry)
- `/lio/pose` - Pose (geometry_msgs/PoseStamped)
- `/lio/current_scan` - Current scan in world frame, GREEN (sensor_msgs/PointCloud2)
- `/lio/map` - Voxel map point cloud, RED (sensor_msgs/PointCloud2)
- `/lio/trajectory` - Trajectory path (nav_msgs/Path)
- `/tf` - TF transform: map → base_link

## Configuration

Config files are located in `lidar_inertial_odometry/config/`:
- `mid360.yaml` - Livox Mid360 configuration
- `avia.yaml` - Livox Avia configuration

These files are automatically installed to `share/lio_ros_wrapper/config/lidar_inertial_odometry/` during build.


## License

This project is released under the MIT License.
