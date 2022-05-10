# Razor IMU ROS2 Driver

ROS2 Driver for Razor IMU (Sparkfun Artemis), a C++ port of the [ROS1 Python Driver](https://github.com/ENSTABretagneRobotics/razor_imu_9dof.git).

Please see the ROS1 driver for details about IMU firmware and calibration process.

# Building

```bash
git clone https://github.com/airacingtech/razor-imu-ros2.git
cd razor-imu-ros2
rosdep install --from-paths razor_imu_ros2/ --ignore-src -y -r
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
```

## ROS Distro Support

The latest branch is `foxy` which would also build in Galactic with warnings.

Accepting PR that updates `declare_parameter` with Galactic practices and removes the TODOs in the code about TF2.

Accepting PR that fixes bugs or adds tests.
