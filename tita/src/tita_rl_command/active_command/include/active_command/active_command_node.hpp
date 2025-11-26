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

#ifndef ACTIVE_COMMAND__ACTIVE_COMMAND_NODE_HPP_
#define ACTIVE_COMMAND__ACTIVE_COMMAND_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "active_command/state_machine.hpp"
#include "active_command/user_btn_function.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "tita_locomotion_interfaces/msg/locomotion_cmd.hpp"
#include "tita_utils/tita_utils.hpp"
#include "tita_utils/topic_names.hpp"

class ActiveCmdNode : public rclcpp::Node
{
public:
  explicit ActiveCmdNode(const rclcpp::NodeOptions & options);

private:
  /// \brief  Numble A receive callback.
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  void user_cmd_callback(const tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr cmd_msg);

  void system_status_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

  void timer_callback();

  double speed_ratio_value(double input);

  double height_value(double input);

  void on_button_change(size_t button_index, float new_state);

  void robot_mode_cb(const std_msgs::msg::String::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fsm_mode_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscription_;
  rclcpp::Subscription<tita_locomotion_interfaces::msg::LocomotionCmd>::SharedPtr
    user_sdk_subscription_;
  rclcpp::Publisher<tita_locomotion_interfaces::msg::LocomotionCmd>::SharedPtr
    active_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr user_msg_;
  tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr teleop_msg_;

  std::shared_ptr<command::StateMachine> state_machine_;
  std::string robot_state_;
  std::vector<float> previous_button_states_;
  std::string fsm_mode_ = "idle";

  double sdk_max_speed_ = 3.0;
  double turn_max_speed_ = 6.0;
  double pitch_max_pose_ = 1.0;
  double roll_max_pose_ = 1.0;
  double height_max_pose_ = 1.0;
  float pub_freq_ = 170.0;
  float body_height_[3] = {0.1, 0.2, 0.3};
  float speed_switch_[3] = {0.3, 0.7, 1.0};
  std::string namespace_ = "";
  bool use_sdk_ = false;
};

#endif  // ACTIVE_COMMAND__ACTIVE_COMMAND_NODE_HPP_
