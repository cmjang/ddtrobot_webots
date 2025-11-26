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

#include "joy_controller/joy_controller.hpp"

namespace joy_controller
{

JoyCommand::JoyCommand(const rclcpp::NodeOptions & options) : Node("joy_controller_node", options)
{
  // RCLCPP_INFO(this->get_logger(), "CommandCanApi device node is started");

  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    tita_topic::joy, subscribers_qos, std::bind(&JoyCommand::joy_cb, this, std::placeholders::_1));

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    tita_topic::manager_twist_command, rclcpp::SystemDefaultsQoS());
  posestamped_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    tita_topic::manager_pose_command, rclcpp::SystemDefaultsQoS());
  fsm_goal_publisher_ = this->create_publisher<std_msgs::msg::String>(
    tita_topic::manager_key_command, rclcpp::SystemDefaultsQoS());
  realtime_cmd_vel_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
      cmd_vel_publisher_);
  realtime_posestamped_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      posestamped_publisher_);
  realtime_fsm_goal_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(fsm_goal_publisher_);
}

void JoyCommand::init(rclcpp::Node::SharedPtr node)
{
  try {
    param_listener_ = std::make_shared<ParamListener>(node);
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return;
  }
}

void JoyCommand::cmd_vel_cb()
{
  if (realtime_cmd_vel_publisher_ && realtime_cmd_vel_publisher_->trylock()) {
    auto & msg = realtime_cmd_vel_publisher_->msg_;
    msg.linear.x = -joy_msg_->axes[2] * params_.twist_linear_ratio;
    msg.angular.z = joy_msg_->axes[3] * params_.twist_angular_ratio;

    realtime_cmd_vel_publisher_->unlockAndPublish();
  }
}

void JoyCommand::posestamped_cb()
{
  if (realtime_posestamped_publisher_ && realtime_posestamped_publisher_->trylock()) {
    auto & msg = realtime_posestamped_publisher_->msg_;
    if (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0) {
      msg.header.stamp = joy_msg_->header.stamp;
    }
    period_ = rclcpp::Time(joy_msg_->header.stamp) - rclcpp::Time(msg.header.stamp);
    // RCLCPP_INFO(this->get_logger(), "Duration is %f seconds", period_.seconds());
    msg.header.stamp = joy_msg_->header.stamp;
    // double roll, pitch, yaw;
    if (joy_msg_->axes[7] == 1) {  // 右switch低位
      roll_ = -joy_msg_->axes[0] * params_.orientation_ratio;
      pitch_ = -joy_msg_->axes[1] * params_.orientation_ratio;
    } else if (joy_msg_->axes[7] == 0) {                // 右switch中位
      y_ = joy_msg_->axes[0] * params_.position_ratio;  // * params_.position_ratio;
      z_ -= joy_msg_->axes[1] * period_.seconds() *
            params_.position_ratio * 4.0 ;  //-joy_msg_->axes[1] * params_.position_ratio;
    } else {                         // 右switch高位
      yaw_ = joy_msg_->axes[0] * params_.orientation_ratio;
    }
    z_ = clamp<double>(z_, 0.2, 0.4);

    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    msg.pose.orientation.x = q.getX();
    msg.pose.orientation.y = q.getY();
    msg.pose.orientation.z = q.getZ();
    msg.pose.orientation.w = q.getW();
    msg.pose.position.x = 0.0;
    msg.pose.position.y = y_;
    msg.pose.position.z = z_;
    realtime_posestamped_publisher_->unlockAndPublish();
  }
  // tf2::Quaternion q(
  //   msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
  //   msg->pose.orientation.w);
  // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // channel_input_.tilt = 0.0;
  // channel_input_.split = 0.0;
  // channel_input_.height = msg->pose.position.z;

  // channel_input_.roll = roll;
  // channel_input_.pitch = pitch;
  // command_api_->send_command_can_channel_input(channel_input_);
}

void JoyCommand::fsm_goal_cb()
{
  // RCLCPP_INFO(
  //   this->get_logger(), "Current state command is %s ", realtime_fsm_goal_publisher_->msg_.data.c_str());

  if (key_msg_updated_) {
    if (realtime_fsm_goal_publisher_ && realtime_fsm_goal_publisher_->trylock()) {
      auto & msg = realtime_fsm_goal_publisher_->msg_;
      if (joy_msg_->axes[4] == 1) {
        msg.data = "idle";
      } else {
        if (joy_msg_->axes[5] == 1) {
          if (msg.data == "balance_stand") {
            msg.data = "transform_down";
          } else if (msg.data == "idle") {
            msg.data = "transform_up";
            std::cout << "transform_up" << std::endl;
          }
        } else if (joy_msg_->axes[5] == -1) {
          msg.data = "rl";
        }
      }
      // if (joy_msg_->axes[4] == -1 && feedback_fsm_ == "idle") {
      //   msg.data = "transform_up";
      //   count++;
      //   if (count > 10) {
      //     msg.data = "balance_stand";
      //     feedback_fsm_ = "balance_stand";
      //     count = 0;
      //   }
      // } else if (joy_msg_->axes[4] == 1 && feedback_fsm_ == "balance_stand") {
      //   msg.data = "transform_down";
      //   count++;
      //   if (count > 20) {
      //     feedback_fsm_ = "idle";
      //     msg.data = "idle";
      //     count = 0;
      //   }
      // }
      // else {
      //   msg.data = "idle";
      // }

      // msg.linear.x = -joy_msg_->axes[2];
      // msg.angular.z = joy_msg_->axes[3];

      realtime_fsm_goal_publisher_->unlockAndPublish();
    }
  }
}

void JoyCommand::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (joy_msg_ == nullptr) {
    joy_msg_ = std::make_shared<sensor_msgs::msg::Joy>(*msg);
  }
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }
  key_msg_updated_ = false;
  if (
    joy_msg_->axes[4] != msg->axes[4] || joy_msg_->axes[5] != msg->axes[5] ||
    joy_msg_->axes[6] != msg->axes[6] || joy_msg_->axes[7] != msg->axes[7]) {
    key_msg_updated_ = true;
  }
  *joy_msg_ = std::move(*msg);

  fsm_goal_cb();
  cmd_vel_cb();
  posestamped_cb();
}

}  // namespace joy_controller
