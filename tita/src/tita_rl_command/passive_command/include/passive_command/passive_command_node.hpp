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

#ifndef PASSIVE_COMMAND__PASSIVE_COMMAND_NODE_HPP_
#define PASSIVE_COMMAND__PASSIVE_COMMAND_NODE_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "passive_command/velocity_control.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tita_locomotion_interfaces/msg/locomotion_cmd.hpp"
#include "tita_utils/topic_names.hpp"

class PassiveCmdNode : public rclcpp::Node
{
public:
  explicit PassiveCmdNode(const rclcpp::NodeOptions & options);

private:
  /// \brief  Numble A receive callback.
  void obstacle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void system_status_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  void timer_callback();

  rclcpp::SyncParametersClient::SharedPtr parameters_client;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  rclcpp::Publisher<tita_locomotion_interfaces::msg::LocomotionCmd>::SharedPtr
    passive_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double pub_freq_ = 170.0;
  double speed_max_ratio_ = 1.0;
  double speed_mid_ratio_ = 0.5;
  double speed_min_ratio_ = 0.15;
  double left_wheel_speed_, right_wheel_speed_;
  double sdk_max_speed_ = 3.0;
  double turn_max_speed_ = 6.0;
  bool obstacle_start_ = true;
  int battery_max_ = 0;
  std::string limit_cmd_ = "None";
  PASSIVE_CMD passive_cmd_;
};

#endif  // PASSIVE_COMMAND__PASSIVE_COMMAND_NODE_HPP_
