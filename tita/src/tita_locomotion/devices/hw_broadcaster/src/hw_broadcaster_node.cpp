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

#include "hw_broadcaster/hw_broadcaster_node.hpp"

using namespace std::chrono_literals;

namespace hw_broadcaster
{

HwBroadcasterNode::HwBroadcasterNode(const rclcpp::NodeOptions & options)
: Node("hw_broadcaster_node", options)
{
  // RCLCPP_INFO(this->get_logger(), "CommandCanApi device node is started");

  const size_t num_joints = joint_names_.size();
  robot_ = std::make_unique<tita_robot>(num_joints);

  rclcpp::QoS info_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  info_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  info_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  info_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(1);

  this->imu_pb_ = this->create_publisher<sensor_msgs::msg::Imu>(tita_topic::body_imu, 1);
  this->joint_states_pb_ =
    this->create_publisher<sensor_msgs::msg::JointState>(tita_topic::joint_states, 1);
  this->motors_status_pb_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(tita_topic::motors_status, 1);
  this->robot_status_pb_ = this->create_publisher<std_msgs::msg::String>(tita_topic::fsm_mode, 1);

  this->realtime_imu_pb_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(imu_pb_);
  this->realtime_joint_states_pb_ =
    std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      joint_states_pb_);
  this->realtime_motors_status_pb_ =
    std::make_shared<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(
      motors_status_pb_);
  this->realtime_robot_status_pb_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(robot_status_pb_);

  auto & joint_state_msg = realtime_joint_states_pb_->msg_;
  joint_state_msg.name = joint_names_;
  joint_state_msg.position.resize(num_joints, kUninitializedValue);
  joint_state_msg.velocity.resize(num_joints, kUninitializedValue);
  joint_state_msg.effort.resize(num_joints, kUninitializedValue);
  auto & motors_status_msg = realtime_motors_status_pb_->msg_;
  motors_status_msg.header.frame_id = "motor_group";
  motors_status_msg.status.resize(1);
  motors_status_msg.status[0].values.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    motors_status_msg.status[0].values[i].key = joint_names_[i];
  }
  this->timer_ = this->create_wall_timer(2ms, std::bind(&HwBroadcasterNode::pub_cb, this));
  this->timer5hz_ = this->create_wall_timer(200ms, std::bind(&HwBroadcasterNode::pub5hz_cb, this));
}

void HwBroadcasterNode::pub5hz_cb()
{
  auto time = this->now();
  if (realtime_motors_status_pb_ && realtime_motors_status_pb_->trylock()) {
    auto & motors_status_msg = realtime_motors_status_pb_->msg_;
    motors_status_msg.header.stamp = time;
    motors_status_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    motors_status_msg.status[0].name = "Motors Status Diagnostic Info";
    motors_status_msg.status[0].message = "Motors Status";

    auto status = robot_->get_joint_status();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      motors_status_msg.status[0].values[i].value = "Online";
      if (status[i] == 0x100) {
        motors_status_msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        motors_status_msg.status[0].values[i].value = "Offline";
      }
    }
    realtime_motors_status_pb_->unlockAndPublish();
  }
  if (realtime_robot_status_pb_ && realtime_robot_status_pb_->trylock()) {
    auto & robot_status_msg = realtime_robot_status_pb_->msg_;
    robot_status_msg.data = robot_->get_robot_status();
    realtime_robot_status_pb_->unlockAndPublish();
  }
}

void HwBroadcasterNode::pub_cb()
{
  auto time = this->now();
  if (realtime_imu_pb_ && realtime_imu_pb_->trylock()) {
    auto & imu_msg = realtime_imu_pb_->msg_;
    imu_msg.header.stamp = time;
    auto orientation = robot_->get_imu_quaternion();
    auto linear_acceleration = robot_->get_imu_acceleration();
    auto angular_velocity = robot_->get_imu_angular_velocity();

    imu_msg.orientation.x = orientation[0];
    imu_msg.orientation.y = orientation[1];
    imu_msg.orientation.z = orientation[2];
    imu_msg.orientation.w = orientation[3];

    imu_msg.angular_velocity.x = angular_velocity[0];
    imu_msg.angular_velocity.y = angular_velocity[1];
    imu_msg.angular_velocity.z = angular_velocity[2];

    imu_msg.linear_acceleration.x = linear_acceleration[0];
    imu_msg.linear_acceleration.y = linear_acceleration[1];
    imu_msg.linear_acceleration.z = linear_acceleration[2];
    realtime_imu_pb_->unlockAndPublish();
  }

  if (realtime_joint_states_pb_ && realtime_joint_states_pb_->trylock()) {
    auto & joint_state_msg = realtime_joint_states_pb_->msg_;
    joint_state_msg.header.stamp = time;
    auto q = robot_->get_joint_q();
    auto v = robot_->get_joint_v();
    auto t = robot_->get_joint_t();
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_state_msg.position[i] = q[i];
      joint_state_msg.velocity[i] = v[i];
      joint_state_msg.effort[i] = t[i];
    }
    realtime_joint_states_pb_->unlockAndPublish();
  }
}

}  // namespace hw_broadcaster
