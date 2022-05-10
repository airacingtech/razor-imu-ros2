# Razor IMU ROS2 Driver

[![ROS2 CI Workflow](https://github.com/airacingtech/razor-imu-ros2/actions/workflows/ros2-ci.yml/badge.svg?branch=foxy)](https://github.com/airacingtech/razor-imu-ros2/actions/workflows/ros2-ci.yml)

ROS2 Driver for Razor IMU (Sparkfun Artemis), a C++ port of the [ROS1 Python Driver](https://github.com/ENSTABretagneRobotics/razor_imu_9dof.git).

Please see the ROS1 driver for details about IMU firmware and calibration process.

## Mounting the IMU

Have the XYZ axis symbols aligned with [REP 103](https://www.ros.org/reps/rep-0103.html). If you are using a Sparkfun OpenLog Artemis, This means the Type-C port is on the right of your robot, and the SD card slot faces down.

# Updating Udev Rules

For your convenience, you can create a `/etc/udev/rules.d/99-razor.rules`:

```bash
KERNEL=="ttyUSB[0-9]*", ACTION=="add", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout", SYMLINK+="sensors/razor"
```

This mounts your IMU to `/dev/sensors/razor`. Re-plug in your IMU to take effect. 

## Building

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
