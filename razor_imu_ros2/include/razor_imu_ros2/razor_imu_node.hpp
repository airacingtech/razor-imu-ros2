// Copyright 2022 AI Racing Tech
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RAZOR_IMU_ROS2__RAZOR_IMU_NODE_HPP_
#define RAZOR_IMU_ROS2__RAZOR_IMU_NODE_HPP_

#include <thread>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "serial_driver/serial_driver.hpp"

using Imu = sensor_msgs::msg::Imu;
using std::placeholders::_1;

namespace razor_imu_ros2
{
class RazorImuNode : public rclcpp::Node
{
public:
  explicit RazorImuNode(const rclcpp::NodeOptions & options);

protected:
  void loop_thread();
  void command(const std::string & command, const double & val, const uint32_t & delay_ms);
  void command(const std::string & command, const uint32_t & delay_ms);

  rclcpp::Publisher<Imu>::SharedPtr m_imu_pub_ {};
  std::unique_ptr<std::thread> m_loop_thread_ {};
  Imu m_imu_ {};
  bool m_enable_offset_;
  bool m_zero_gravity_;
  tf2::Quaternion m_q_offset_;

// Serial Driver
  std::unique_ptr<drivers::common::IoContext> owned_ctx {};
  std::unique_ptr<drivers::serial_driver::SerialDriver> driver_ {};
};
}  // namespace razor_imu_ros2

#endif  // RAZOR_IMU_ROS2__RAZOR_IMU_NODE_HPP_
