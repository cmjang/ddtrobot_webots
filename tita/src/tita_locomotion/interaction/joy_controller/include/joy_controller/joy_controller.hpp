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

#ifndef JOY_COMMAND__JOY_COMMAND_HPP_
#define JOY_COMMAND__JOY_COMMAND_HPP_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "joy_controller_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "tita_utils/topic_names.hpp"

namespace joy_controller
{
class JoyCommand : public rclcpp::Node
{
public:
  explicit JoyCommand(const rclcpp::NodeOptions & option);
  ~JoyCommand() = default;
  void init(rclcpp::Node::SharedPtr node);

private:  
  template <typename scalar_t>
  scalar_t clamp(scalar_t value, scalar_t min, scalar_t max)
  {
    return std::max(min, std::min(value, max));
  }
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void cmd_vel_cb();
  void posestamped_cb();
  void fsm_goal_cb();
  void param_cb();

  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Joy>> joy_subscription_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> cmd_vel_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> posestamped_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> fsm_goal_publisher_;

  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>
    realtime_cmd_vel_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
    realtime_posestamped_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>>
    realtime_fsm_goal_publisher_;
  
  double roll_{0}, pitch_{0}, yaw_{0}, x_{0}, y_{0}, z_{0.1};
  rclcpp::Duration period_ = rclcpp::Duration::from_seconds(0.0);
  sensor_msgs::msg::Joy::SharedPtr joy_msg_ = nullptr;
  bool key_msg_updated_{false};
  std::string command_fsm_{"idle"}, feedback_fsm_{"idle"};
  long int count{0};

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};
}  // namespace joy_controller

#endif  // JOY_COMMAND__JOY_COMMAND_HPP_
