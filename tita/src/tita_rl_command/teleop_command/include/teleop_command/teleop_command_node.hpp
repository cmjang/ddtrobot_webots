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

#ifndef TELEOP_COMMAND__TELEOP_COMMAND_NODE_HPP_
#define TELEOP_COMMAND__TELEOP_COMMAND_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "teleop_command/button_function.hpp"
#include "teleop_command/elrs_emwave_device.hpp"
#include "tita_utils/topic_names.hpp"

typedef struct
{
  uint32_t version;
  uint32_t error_code;
  uint8_t stop_obst_mode : 1;
  uint8_t use_sdk_mode : 1;
  uint8_t bypass_obst_mode : 1;
  uint8_t speed_limit_mode : 1;
  uint8_t car_control_mode : 1;
  uint8_t auto_mapping_mode : 1;
  uint8_t hotspot_mode : 1;
  uint8_t reserved : 1;
  uint8_t bat_left;
  uint8_t bat_right;
  uint8_t ip_end;
} TransferDataT;

typedef enum {
  TITA_WIFI_CONNECT_MODE = -100,
  TITA_STOP_OBST_MODE = 1,
  TITA_USE_SDK_MODE = 2,
  TITA_BYPASS_OBST_MODE = 3,
  TITA_SPEED_LIMIT_MODE = 4,
  TITA_IMAGE_TRAN_MODE = 5,
  TITA_AUTO_MAPPING_MODE = 6,
  TITA_HOTSPOT_MODE = 7,
} tita_mode_switch;

class TeleopCmdNode : public rclcpp::Node
{
public:
  explicit TeleopCmdNode(const rclcpp::NodeOptions & options);

private:
  /// \brief  Subscribe joy msg and publish new joy msg.
  /// \param  Joy msg.
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  void mode_switch_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  bool waitForService(
    const rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
    const std::string & node_name);

  void set_parameter(bool mark);

  /// \brief  Subscribe joy msg and map button.
  /// \param  User Joy msg.
  /// \param  Controller Joy msg.
  /// \param  Param msg.
  /// \param  Controller joy btn_index.

  void processJoyMessage(
    const sensor_msgs::msg::Joy::SharedPtr & joy_msg,
    std::unique_ptr<sensor_msgs::msg::Joy> & unique_joy_msg, std::string mark, int btn_index);

  // rclcpp::AsyncParametersClient::SharedPtr obstacle_param_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr sdk_param_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr obstacle_param_client_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr lbattery_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr rbattery_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_process_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int _AxesSize = 8;
  float _AxesBias = 0.0013;
  bool _DebugMode = false;
  std::string _MoveAxes = "a0";
  std::string _TurnAxes = "a1";
  std::string _RollAxes = "a2";
  std::string _PitchAxes = "a3";
  std::string _StandUp = "b4";
  std::string _HeightSwitch = "b5";
  std::string _JumpButton = "b6";
  std::string _SpeedButton = "b7";
  std::string namespace_ = "";

  sensor_msgs::msg::BatteryState lbattery_info, rbattery_info;
  CRSF crsf;
  TransferDataT transfer_data;
  int last_key;
  int wifi_key_offset;
  int wifi_data[34];
  std::string exec(const char * cmd);
  int getWlan0IpLastByte(const std::string & ipAddrOutput);
  void status_timer_callback(void);
  void ModeReadcallback(const uint8_t data[]);
  void DataReadCallBack(const uint16_t channels[]);
  float normalizeValue(float originalValue, float minOriginal, float rangeOriginal);
  uint32_t date_to_uint32(const char * date_str);
  void set_parameter(
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client,
    std::string param_name, bool mark);
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr elrs_publisher_;
  //=======================
};

#endif  // TELEOP_COMMAND__TELEOP_COMMAND_NODE_HPP_
