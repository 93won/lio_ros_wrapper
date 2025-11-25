# LIO ROS Wrapper

ROS2 wrapper for [lidar_inertial_odometry](https://github.com/93won/lidar_inertial_odometry).

### MIT License

## Demo Video

[![LIO ROS Wrapper Demo](https://img.youtube.com/vi/AlfKASD0Rrc/0.jpg)](https://youtu.be/AlfKASD0Rrc)

**Watch on YouTube**: https://youtu.be/AlfKASD0Rrc

## Installation

```bash
# Clone with submodules
cd ~/ros2_ws/src
git clone --recursive https://github.com/93won/lio_ros_wrapper.git
cd lio_ros_wrapper

# If already cloned without --recursive
git submodule update --init --recursive

# Install dependencies
sudo apt install -y libeigen3-dev libyaml-cpp-dev libspdlog-dev

# Build
cd ~/ros2_ws
colcon build --packages-select lio_ros_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Usage

```bash
# Launch with RViz (Livox AVIA)
ros2 launch lio_ros_wrapper lio_avia.launch.py

# Launch with RViz (Livox Mid360)
ros2 launch lio_ros_wrapper lio_mid360.launch.py
```

## Example Dataset

Download ROS2 bag files from:  
**https://drive.google.com/drive/folders/1uqa_LDlOTtQo1PIBcXQ22JObDuHHwBAs?usp=sharing**

```bash
# Play bag file
ros2 bag play your_dataset.db3 -r 1.0

# Play at 10x speed
ros2 bag play your_dataset.db3 -r 10
```

## License

MIT License
