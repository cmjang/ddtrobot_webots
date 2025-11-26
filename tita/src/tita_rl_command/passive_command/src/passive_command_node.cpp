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

#include "passive_command/passive_command_node.hpp"

PassiveCmdNode::PassiveCmdNode(const rclcpp::NodeOptions & options)
: Node("passive_command_node", options)
{
  parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "teleop_command_node");
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for parameter service to be available...");
  }
  auto parameters = parameters_client->get_parameters(
    {"speed_max_ratio", "speed_mid_ratio", "speed_min_ratio", "sdk_max_speed", "turn_max_speed",
     "pub_freq"});
  for (const auto & param : parameters) {
    if (param.get_name() == "pub_freq") {
      pub_freq_ = param.as_double();
    } else if (param.get_name() == "speed_max_ratio") {
      sdk_max_speed_ = param.as_double();
    } else if (param.get_name() == "speed_mid_ratio") {
      speed_mid_ratio_ = param.as_double();
    } else if (param.get_name() == "speed_min_ratio") {
      speed_min_ratio_ = param.as_double();
    } else if (param.get_name() == "sdk_max_speed") {
      sdk_max_speed_ = param.as_double();
    } else if (param.get_name() == "turn_max_speed") {
      turn_max_speed_ = param.as_double();
    }
  }
  passive_cmd_.max_speed = sdk_max_speed_;
  passive_cmd_.max_turn = turn_max_speed_;

  obstacle_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    tita_topic::obstacle_data, 10,
    std::bind(&PassiveCmdNode::obstacle_callback, this, std::placeholders::_1));

  passive_cmd_publisher_ = this->create_publisher<tita_locomotion_interfaces::msg::LocomotionCmd>(
    tita_topic::passive_command, 10);

  vel_pub_ = this->create_publisher<std_msgs::msg::Float64>("robot_vel", 10);

  joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    tita_topic::joint_states, 10,
    std::bind(&PassiveCmdNode::joint_callback, this, std::placeholders::_1));

  diagnostic_subscription_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    tita_topic::system_status, 10,
    std::bind(&PassiveCmdNode::system_status_callback, this, std::placeholders::_1));

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&PassiveCmdNode::parameter_callback, this, std::placeholders::_1));

  auto period = std::chrono::duration<double>(1.0 / pub_freq_);
  timer_ = this->create_wall_timer(period, std::bind(&PassiveCmdNode::timer_callback, this));
}

void PassiveCmdNode::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "joint_left_leg_4") {
      left_wheel_speed_ = msg->velocity[i];
    } else if (msg->name[i] == "joint_right_leg_4") {
      right_wheel_speed_ = msg->velocity[i];
    }
  }
}

void PassiveCmdNode::obstacle_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  if (msg->data < 0.4) {
    passive_cmd_.max_speed = 0.0;
  } else if (msg->data > 1.49) {
    passive_cmd_.max_speed = sdk_max_speed_;
  } else {
    passive_cmd_.max_speed = velocity_remap(msg->data);
  }
  // auto unique_joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
  // *unique_joy_msg = *joy_msg;
  // joy_publisher_->publish(std::move(unique_joy_msg));
}

void PassiveCmdNode::system_status_callback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for (const auto & status : msg->status) {
    for (const auto & value : status.values) {
      if (value.key == "PowerStatus") {
        if (value.value == "POWERLOW") {
          limit_cmd_ = "NoJump";
        } else if (value.value == "POWERZERO") {
          limit_cmd_ = "NoBalance";
        } else if (value.value == "POWEROFF") {
          limit_cmd_ = "ShutDown";
        } else {
          limit_cmd_ = "None";
        }
      }
    }
  }
  for (const auto & status : msg->status) {
    for (const auto & value : status.values) {
      if (value.key == "Robot") {
        if (value.value == "Timeout" && limit_cmd_ == "ShutDown") {
          limit_cmd_ = "OffLine";
        } 
      }
    }
  }
}

void PassiveCmdNode::timer_callback()
{
  auto unique_joy_msg = std::make_unique<tita_locomotion_interfaces::msg::LocomotionCmd>();
  unique_joy_msg->header.stamp = this->now();
  unique_joy_msg->header.frame_id = "passive_joy";

  auto average_speed = ((left_wheel_speed_ + right_wheel_speed_) * 0.0925) / 2.0;
  auto speed_msg = std::make_unique<std_msgs::msg::Float64>();
  speed_msg->data = average_speed;
  vel_pub_->publish(std::move(speed_msg));
  if (!obstacle_start_) {
    passive_cmd_.max_speed = sdk_max_speed_;
  }
  unique_joy_msg->twist.linear.x = passive_cmd_.max_speed;
  unique_joy_msg->fsm_mode = limit_cmd_;
  passive_cmd_publisher_->publish(std::move(unique_joy_msg));
}

rcl_interfaces::msg::SetParametersResult PassiveCmdNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "obstacle_start") {
      obstacle_start_ = parameter.as_bool();
      RCLCPP_INFO(this->get_logger(), "The value is: %s", obstacle_start_ ? "true" : "false");
    }
  }
  return result;
}
