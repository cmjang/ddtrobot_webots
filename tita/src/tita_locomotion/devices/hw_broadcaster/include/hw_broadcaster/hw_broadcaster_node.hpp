// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

#ifndef HW_BROADCASTER__HW_BROADCASTER_NODE_HPP_
#define HW_BROADCASTER__HW_BROADCASTER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tita_robot/tita_robot.hpp"
#include "tita_utils/topic_names.hpp"

namespace hw_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();

class HwBroadcasterNode : public rclcpp::Node
{
public:
  explicit HwBroadcasterNode(const rclcpp::NodeOptions & option);
  ~HwBroadcasterNode() = default;

private:
  void pub_cb();
  void pub5hz_cb();
  rclcpp::TimerBase::SharedPtr timer_, timer5hz_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> imu_pb_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_states_pb_;
  std::shared_ptr<rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>> motors_status_pb_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> robot_status_pb_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> realtime_imu_pb_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
    realtime_joint_states_pb_;
  std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>
    realtime_motors_status_pb_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>>
    realtime_robot_status_pb_;

private:
  std::unique_ptr<tita_robot> robot_;
  std::vector<std::string> joint_names_ = {
    "joint_left_leg_1",  "joint_left_leg_2",  "joint_left_leg_3",  "joint_left_leg_4",
    "joint_right_leg_1", "joint_right_leg_2", "joint_right_leg_3", "joint_right_leg_4"};
};
}  // namespace hw_broadcaster

#endif  // HW_BROADCASTER__HW_BROADCASTER_NODE_HPP_
