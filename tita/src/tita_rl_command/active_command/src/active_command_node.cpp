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

#include "active_command/active_command_node.hpp"

ActiveCmdNode::ActiveCmdNode(const rclcpp::NodeOptions & options)
: Node("active_command_node", options)
{
  this->get_parameter("use_sdk", use_sdk_);
  this->get_parameter("sdk_max_speed", sdk_max_speed_);
  this->get_parameter("turn_max_speed", turn_max_speed_);
  this->get_parameter("pitch_max_pose", pitch_max_pose_);
  this->get_parameter("roll_max_pose", roll_max_pose_);
  this->get_parameter("height_max_pose", height_max_pose_);
  this->get_parameter("pub_freq", pub_freq_);
  this->get_parameter("speed_max_ratio", speed_switch_[2]);
  this->get_parameter("speed_mid_ratio", speed_switch_[1]);
  this->get_parameter("speed_min_ratio", speed_switch_[0]);
  this->get_parameter("height_max", body_height_[2]);
  this->get_parameter("height_mid", body_height_[1]);
  this->get_parameter("height_min", body_height_[0]);
  namespace_ = this->get_namespace();
  if (!namespace_.empty() && namespace_[0] == '/') {
    namespace_ = namespace_.substr(1);
  }

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ActiveCmdNode::parameter_callback, this, std::placeholders::_1));

  user_msg_ = std::make_shared<tita_locomotion_interfaces::msg::LocomotionCmd>();
  teleop_msg_ = std::make_shared<tita_locomotion_interfaces::msg::LocomotionCmd>();
  state_machine_ = std::make_shared<command::StateMachine>();

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    tita_topic::teleop_command, 10,
    std::bind(&ActiveCmdNode::joy_callback, this, std::placeholders::_1));

  user_sdk_subscription_ =
    this->create_subscription<tita_locomotion_interfaces::msg::LocomotionCmd>(
      tita_topic::user_command, 10,
      std::bind(&ActiveCmdNode::user_cmd_callback, this, std::placeholders::_1));

  fsm_mode_subscription_ = this->create_subscription<std_msgs::msg::String>(
    tita_topic::fsm_mode, 10,
    std::bind(&ActiveCmdNode::robot_mode_cb, this, std::placeholders::_1));


  diagnostic_subscription_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    tita_topic::system_status, 10,
    std::bind(&ActiveCmdNode::system_status_callback, this, std::placeholders::_1));

  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST).keep_last(1);

  active_cmd_publisher_ = this->create_publisher<tita_locomotion_interfaces::msg::LocomotionCmd>(
    tita_topic::active_command, qos);

  auto period = std::chrono::duration<double>(1.0 / pub_freq_);
  timer_ = this->create_wall_timer(period, std::bind(&ActiveCmdNode::timer_callback, this));
}

void ActiveCmdNode::system_status_callback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  for (const auto & status : msg->status) {
    for (const auto & value : status.values) {
      if (value.key == "Joy") {
        if (value.value == "Timeout") {
          auto teleop_msg = std::make_shared<tita_locomotion_interfaces::msg::LocomotionCmd>();
          teleop_msg_ = teleop_msg;
        }
      }
    }
  }
}

double ActiveCmdNode::speed_ratio_value(double input)
{
  return input == 1.0    ? speed_switch_[0]
         : input == 0.0  ? speed_switch_[1]
         : input == -1.0 ? speed_switch_[2]
                         : 0.0;  // default
}

double ActiveCmdNode::height_value(double input)
{
  return input == 1.0    ? body_height_[0]
         : input == 0.0  ? body_height_[1]
         : input == -1.0 ? body_height_[2]
                         : 0.0;  // default
}

void ActiveCmdNode::on_button_change(size_t button_index, float new_state)
{
  if (button_index == 4) {
    if (new_state == -1.0) {
      if (fsm_mode_ == "idle") state_machine_->handle_event(command::Event::EVENT_IDLE_TRANSUP);
    }
    if (new_state == 1.0) {
      if (fsm_mode_ == "balance")
        state_machine_->handle_event(command::Event::EVENT_BALANCE_TRANSDOWN);
    }
  }
  if (button_index == 6) {
    if (new_state == -1.0 && fsm_mode_ == "balance") {
      state_machine_->handle_event(command::Event::EVENT_BALANCE_CHARGE);
      // std::this_thread::sleep_for(std::chrono::milliseconds(210));
    }
    if (new_state == 1.0 && fsm_mode_ == "charge") {
      state_machine_->handle_event(command::Event::EVENT_CHARGE_JUMP);
      // std::this_thread::sleep_for(std::chrono::milliseconds(210));
    }
    // RCLCPP_INFO(this->get_logger(), "current_button_state: %.2f previous_button_states: %s , %s", new_state,fsm_mode_.c_str(),(state_machine_->get_string_state()).c_str());
  }
}

void ActiveCmdNode::robot_mode_cb(const std_msgs::msg::String::SharedPtr msg)
{
  fsm_mode_ = msg->data.c_str();
  command::State robot_state = state_machine_->string2state(fsm_mode_);
  state_machine_->set_state(robot_state);
}

void ActiveCmdNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  float speed_ratio = speed_ratio_value(joy_msg->axes[7]);

  if (previous_button_states_.empty()) {
    previous_button_states_.resize(joy_msg->axes.size(), 0);
  }

  for (size_t i = 0; i < joy_msg->axes.size(); ++i) {
    if (i > 3) {
      float current_button_state = joy_msg->axes[i];
      if (i == 4) on_button_change(i, current_button_state);
      if (i == 6) on_button_change(i, current_button_state);

      // if (current_button_state != previous_button_states_[i] && i == 4) {
        // on_button_change(i, current_button_state);
        // previous_button_states_[i] = current_button_state;
      // }
    }
  }

  teleop_msg_->header = joy_msg->header;
  teleop_msg_->twist.linear.x = mapToRange(
    -1.0 * joy_msg->axes[2], -1.0 * sdk_max_speed_ * speed_ratio, sdk_max_speed_ * speed_ratio);

  teleop_msg_->twist.angular.z = mapToRange(
    joy_msg->axes[3], -1.0 * turn_max_speed_ * speed_ratio, turn_max_speed_ * speed_ratio);

  auto joy_pitch = tita_utils::pitchToQuaternion(
    mapToRange(-1.0 * joy_msg->axes[1], -1.0 * pitch_max_pose_, pitch_max_pose_));

  auto joy_roll = tita_utils::rollToQuaternion(
    mapToRange(-1.0 * joy_msg->axes[0], -1.0 * roll_max_pose_, roll_max_pose_));

  auto joy_body_pose = tita_utils::multiplyQuaternions(joy_pitch, joy_roll);
  teleop_msg_->pose.orientation.w = joy_body_pose.w;
  teleop_msg_->pose.orientation.x = joy_body_pose.x;
  teleop_msg_->pose.orientation.y = joy_body_pose.y;
  teleop_msg_->pose.orientation.z = joy_body_pose.z;

  teleop_msg_->pose.position.z = height_value(joy_msg->axes[5]);
  // active_cmd_publisher_->publish(std::move(unique_cmd_msg));
}

void ActiveCmdNode::user_cmd_callback(
  const tita_locomotion_interfaces::msg::LocomotionCmd::SharedPtr cmd_msg)
{
  *user_msg_ = *cmd_msg;
}

void ActiveCmdNode::timer_callback()
{
  auto unique_joy_msg = std::make_unique<tita_locomotion_interfaces::msg::LocomotionCmd>();
  if (use_sdk_) {
    *unique_joy_msg = *user_msg_;
  } else {
    *unique_joy_msg = *teleop_msg_;
    *user_msg_ = *teleop_msg_;
    unique_joy_msg->fsm_mode = state_machine_->get_string_state();
  }
  active_cmd_publisher_->publish(std::move(unique_joy_msg));
}

rcl_interfaces::msg::SetParametersResult ActiveCmdNode::parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "use_sdk") {
      use_sdk_ = parameter.as_bool();
    }
  }
  return result;
}
