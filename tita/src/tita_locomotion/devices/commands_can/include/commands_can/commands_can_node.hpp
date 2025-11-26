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

#ifndef COMMANDS_CAN__COMMANDS_CAN_NODE_HPP_
#define COMMANDS_CAN__COMMANDS_CAN_NODE_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "tita_robot/tita_robot.hpp"
#include "tita_utils/topic_names.hpp"

namespace commands_can
{
class CommandCanNode : public rclcpp::Node
{
public:
  explicit CommandCanNode(const rclcpp::NodeOptions & option);
  ~CommandCanNode() = default;

private:
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void fsm_goal_cb(const std_msgs::msg::String::SharedPtr msg);
  // void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void set_tilt_cb(const std_msgs::msg::Float64::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr posestamped_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_goal_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_tilt_subscription_;

private:
  std::unique_ptr<tita_robot> robot_;
  can_device::api_channel_input_t channel_input_;
  std_msgs::msg::String::SharedPtr fsm_mode_msg_ = nullptr;
};
}  // namespace commands_can

#endif  // COMMANDS_CAN__COMMANDS_CAN_NODE_HPP_
