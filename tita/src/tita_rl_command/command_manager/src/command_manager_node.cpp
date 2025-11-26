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

#include "command_manager/command_manager_node.hpp"

CommandManagerNode::CommandManagerNode(const rclcpp::NodeOptions & options)
: Node("command_manager_node", options)
{
  // this->get_parameter("value1", value1_);
  // this->get_parameter("value2", value2_);
  // RCLCPP_INFO(this->get_logger(), "value1 : '%f'", value1_);
  power_off_mark_ = false;
  active_cmd_subscription_ =
    this->create_subscription<tita_locomotion_interfaces::msg::LocomotionCmd>(
      tita_topic::active_command, 10,
      std::bind(&CommandManagerNode::active_callback, this, std::placeholders::_1));
  passive_cmd_subscription_ =
    this->create_subscription<tita_locomotion_interfaces::msg::LocomotionCmd>(
      tita_topic::passive_command, 10,
      std::bind(&CommandManagerNode::passive_callback, this, std::placeholders::_1));
  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    tita_topic::teleop_command, 10,
    std::bind(&CommandManagerNode::joy_callback, this, std::placeholders::_1));
  fsm_mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
    tita_topic::fsm_mode, 10,
    std::bind(&CommandManagerNode::robot_fsm_callback, this, std::placeholders::_1));

  body_pose_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(tita_topic::manager_pose_command, 10);
  body_twist_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>(tita_topic::manager_twist_command, 10);
  body_fsm_publisher_ =
    this->create_publisher<std_msgs::msg::String>(tita_topic::manager_key_command, 10);

  audio_client_ =
    this->create_client<tita_system_interfaces::srv::PlayAudioSystemPrompts>(tita_topic::kplay_audio_system_prompts);
  power_off_client_ =
    this->create_client<std_srvs::srv::Trigger>(tita_topic::krobot_power_off_trigger_service);
  audio_sound_client_ =
    this->create_client<rcl_interfaces::srv::SetParameters>("audio_controller_node/set_parameters");
}

void CommandManagerNode::active_callback(
  const tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr cmd_msg)
{
  auto pose_message = std::make_unique<geometry_msgs::msg::PoseStamped>();
  pose_message->header.stamp = cmd_msg->header.stamp;
  pose_message->header.frame_id = cmd_msg->header.frame_id;
  pose_message->pose = cmd_msg->pose;

  auto twist_message = std::make_unique<geometry_msgs::msg::Twist>();
  *twist_message = cmd_msg->twist;
  auto fsm_message = std::make_shared<std_msgs::msg::String>();
  fsm_message->data = cmd_msg->fsm_mode;

  if (passive_msgs_) {
    if (twist_message->linear.x > passive_msgs_->twist.linear.x) {
      twist_message->linear.x = passive_msgs_->twist.linear.x;
    }
    if (passive_msgs_->fsm_mode == "NoJump") {
      if (fsm_message->data == "charge" || fsm_message->data == "jump") {
        fsm_message->data = "balance";
      }
    } else if (passive_msgs_->fsm_mode == "NoBalance" && 
        (robot_fsm_mode_ == "balance" || robot_fsm_mode_ == "idle")) {
      fsm_message->data = "transform_down";
    } else if (passive_msgs_->fsm_mode == "ShutDown" && robot_fsm_mode_ == "balance") {
      fsm_message->data = "transform_down";
    } else if (passive_msgs_->fsm_mode == "ShutDown" && robot_fsm_mode_ == "transform_down") {
      fsm_message->data = "transform_down";
    } else if (
      passive_msgs_->fsm_mode == "ShutDown" && robot_fsm_mode_ == "idle" &&
      power_off_mark_ == false) {

      if (!power_off_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available");
        return;
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      power_off_client_->async_send_request(request);
      power_off_mark_ = true;
    } else if(power_off_mark_ == true){
      robot_fsm_mode_ = "idle";
    } else if (
      passive_msgs_->fsm_mode == "OffLine" && power_off_mark_ == false) {
        
      if (!power_off_client_->wait_for_service(std::chrono::seconds(3))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available");
        return;
      }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      power_off_client_->async_send_request(request);
      power_off_mark_ = true;
    }
  }
  if (stop_emergency == "stop") {
    fsm_message->data = "stop";
  }
  body_pose_publisher_->publish(std::move(pose_message));
  body_twist_publisher_->publish(std::move(twist_message));
  body_fsm_publisher_->publish(*fsm_message);
}

void CommandManagerNode::passive_callback(
  const tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr passive_cmd)
{
  passive_msgs_ = passive_cmd;
}

void CommandManagerNode::robot_fsm_callback(const std_msgs::msg::String::SharedPtr msg)
{
  robot_fsm_mode_ = msg->data;
}

void CommandManagerNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  if (
    joy_msg->axes[4] == 1.0 && joy_msg->axes[5] == 1.0 && joy_msg->axes[6] == -1.0 &&
    joy_msg->axes[7] == 1.0) {
    stop_emergency = "stop";
  } else {
    stop_emergency = "None";
  }
}