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

#include "commands_can/commands_can_node.hpp"

namespace commands_can
{

CommandCanNode::CommandCanNode(const rclcpp::NodeOptions & options)
: Node("command_can_node", options)
{
  RCLCPP_INFO(this->get_logger(), "CommandCanApi device node is started");
  robot_ = std::make_unique<tita_robot>(8);

  rclcpp::QoS info_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  info_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  info_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(1);

  cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    tita_topic::manager_twist_command, info_qos,
    std::bind(&CommandCanNode::cmd_vel_cb, this, std::placeholders::_1));
  posestamped_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    tita_topic::manager_pose_command, info_qos,
    std::bind(&CommandCanNode::posestamped_cb, this, std::placeholders::_1));
  fsm_goal_subscription_ = this->create_subscription<std_msgs::msg::String>(
    tita_topic::manager_key_command, info_qos,
    std::bind(&CommandCanNode::fsm_goal_cb, this, std::placeholders::_1));
  // joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
  //   tita_topic::teleop_command, info_qos,
  //   std::bind(&CommandCanNode::joy_cb, this, std::placeholders::_1));
  set_tilt_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "robot_inertia_calculator/set_tilt", info_qos,
    std::bind(&CommandCanNode::set_tilt_cb, this, std::placeholders::_1));
  channel_input_.tilt = 100;  // for distinguish use mcu tilt or soc tilt
}

void CommandCanNode::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  channel_input_.forward = msg->linear.x;
  channel_input_.yaw = msg->angular.z;
}

void CommandCanNode::posestamped_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion q(
    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
    msg->pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // channel_input_.tilt = 0.0;
  channel_input_.split = 0.0;
  channel_input_.height = msg->pose.position.z;

  channel_input_.roll = roll;
  channel_input_.pitch = pitch;
  robot_->set_rc_input(channel_input_);
}

void CommandCanNode::fsm_goal_cb(const std_msgs::msg::String::SharedPtr msg)
{
  if (fsm_mode_msg_ == nullptr) {
    fsm_mode_msg_ = std::make_shared<std_msgs::msg::String>();
    *fsm_mode_msg_ = *msg;
  }

  // RCLCPP_INFO(this->get_logger(), "command: %s", msg->data.c_str());

  if (fsm_mode_msg_->data != msg->data) {
    if (msg->data == "transform_up") {
      robot_->set_robot_stand(true);
    } else if (msg->data == "transform_down") {
      robot_->set_robot_stand(false);
    }

    if (msg->data == "charge") {
      robot_->set_robot_jump(true);
    } else if (msg->data == "jump") {
      robot_->set_robot_jump(false);
    }
    if (msg->data == "stop") {
      robot_->set_robot_stop();
    }
  }
  *fsm_mode_msg_ = std::move(*msg);
}

void CommandCanNode::set_tilt_cb(const std_msgs::msg::Float64::SharedPtr msg)
{
  (void)msg;
  channel_input_.tilt = msg->data;
}

}  // namespace commands_can
