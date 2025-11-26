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

#ifndef COMMAND_MANAGER__COMMAND_MANAGER_NODE_HPP_
#define COMMAND_MANAGER__COMMAND_MANAGER_NODE_HPP_

#include <chrono>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tita_locomotion_interfaces/msg/locomotion_cmd.hpp"
#include "tita_system_interfaces/srv/play_audio_system_prompts.hpp"
#include "tita_utils/topic_names.hpp"

class CommandManagerNode : public rclcpp::Node
{
public:
  explicit CommandManagerNode(const rclcpp::NodeOptions & options);

private:
  void active_callback(const tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr cmd_msg);

  void passive_callback(
    const tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr passive_cmd);

  void robot_fsm_callback(const std_msgs::msg::String::SharedPtr msg);

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  rclcpp::Subscription<tita_locomotion_interfaces::msg::LocomotionCmd>::SharedPtr
    active_cmd_subscription_;
  rclcpp::Subscription<tita_locomotion_interfaces::msg::LocomotionCmd>::SharedPtr
    passive_cmd_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_mode_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr body_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr body_twist_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr body_fsm_publisher_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr audio_sound_client_;
  rclcpp::Client<tita_system_interfaces::srv::PlayAudioSystemPrompts>::SharedPtr audio_client_;

  rclcpp::TimerBase::SharedPtr timer_;
  tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr passive_msgs_;
  std::string robot_fsm_mode_ = "idle";
  std::string stop_emergency = "None";
  bool power_off_mark_ = false;
  double value1_;
  double value2_;
};

#endif  // COMMAND_MANAGER__COMMAND_MANAGER_NODE_HPP_
