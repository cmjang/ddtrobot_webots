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

#include "teleop_command/teleop_command_node.hpp"

TeleopCmdNode::TeleopCmdNode(const rclcpp::NodeOptions & options)
: Node("teleop_command_node", options)
{
  this->get_parameter("DebugMode", _DebugMode);
  this->get_parameter("AxesSize", _AxesSize);
  this->get_parameter("MoveAxes", _MoveAxes);
  this->get_parameter("TurnAxes", _TurnAxes);
  this->get_parameter("PitchAxes", _PitchAxes);
  this->get_parameter("RollAxes", _RollAxes);
  this->get_parameter("HeightSwitch", _HeightSwitch);
  this->get_parameter("SpeedButton", _SpeedButton);
  this->get_parameter("StandUp", _StandUp);
  this->get_parameter("JumpButton", _JumpButton);
  this->get_parameter("AxesBias", _AxesBias);
  this->transfer_data.stop_obst_mode = 1;
  this->transfer_data.use_sdk_mode = 0;

  namespace_ = this->get_namespace();
  if (!namespace_.empty() && namespace_[0] == '/') {
    namespace_ = namespace_.substr(1);
  }
  obstacle_param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
    "obstacle_detector_node/set_parameters");
  sdk_param_client_ =
    this->create_client<rcl_interfaces::srv::SetParameters>("active_command_node/set_parameters");

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
    tita_topic::joy, 10, std::bind(&TeleopCmdNode::joy_callback, this, std::placeholders::_1));

  lbattery_subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    tita_topic::kleft_battery_topic, 10,
    std::bind(&TeleopCmdNode::batteryCallback, this, std::placeholders::_1));

  rbattery_subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    tita_topic::kright_battery_topic, 10,
    std::bind(&TeleopCmdNode::batteryCallback, this, std::placeholders::_1));

  joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(tita_topic::teleop_command, 10);

  elrs_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(tita_topic::joy, 10);

  if (!_DebugMode) {
    crsf.onDataReceived(std::bind(&TeleopCmdNode::DataReadCallBack, this, std::placeholders::_1));
    crsf.onModeReceived(std::bind(&TeleopCmdNode::ModeReadcallback, this, std::placeholders::_1));
    crsf.begin();
    joy_subscription_process_ = this->create_subscription<sensor_msgs::msg::Joy>(
      tita_topic::joy, 10,
      std::bind(&TeleopCmdNode::mode_switch_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200), [this]() { this->status_timer_callback(); });
  }
}

void TeleopCmdNode::batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  if (msg->header.frame_id == namespace_ + "/left_battery_info") {
    lbattery_info = *msg;
  }
  if (msg->header.frame_id == namespace_ + "/right_battery_info") {
    rbattery_info = *msg;
  }
}

void TeleopCmdNode::processJoyMessage(
  const sensor_msgs::msg::Joy::SharedPtr & joy_msg,
  std::unique_ptr<sensor_msgs::msg::Joy> & unique_joy_msg, std::string mark, int btn_index)
{
  auto result = checkAndExtract(mark);

  if (std::get<0>(result) == 0) {
    unique_joy_msg->axes[btn_index] = joy_msg->axes[std::get<1>(result)];
  } else if (std::get<0>(result) == 1) {
    unique_joy_msg->axes[btn_index] = mapToButtonValue(joy_msg->axes[std::get<1>(result)]);
  } else {
    unique_joy_msg->axes[btn_index] = 0.0;
  }
}

void TeleopCmdNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  auto unique_joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
  unique_joy_msg->header = joy_msg->header;
  unique_joy_msg->axes.resize(_AxesSize, 0);
  processJoyMessage(joy_msg, unique_joy_msg, _MoveAxes, 0);
  processJoyMessage(joy_msg, unique_joy_msg, _TurnAxes, 1);
  processJoyMessage(joy_msg, unique_joy_msg, _PitchAxes, 2);
  processJoyMessage(joy_msg, unique_joy_msg, _RollAxes, 3);
  processJoyMessage(joy_msg, unique_joy_msg, _StandUp, 4);
  processJoyMessage(joy_msg, unique_joy_msg, _HeightSwitch, 5);
  processJoyMessage(joy_msg, unique_joy_msg, _JumpButton, 6);
  processJoyMessage(joy_msg, unique_joy_msg, _SpeedButton, 7);

  joy_publisher_->publish(std::move(unique_joy_msg));
}

std::string TeleopCmdNode::exec(const char * cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

int TeleopCmdNode::getWlan0IpLastByte(const std::string & ipAddrOutput)
{
  std::istringstream iss(ipAddrOutput);
  std::string line;
  while (std::getline(iss, line)) {
    if (line.find("wlan0") != std::string::npos && line.find("inet ") != std::string::npos) {
      size_t start = line.find("inet ") + 5;
      size_t end = line.find_first_of(" ", start);
      std::string ip = line.substr(start, end - start);
      std::istringstream ipStream(ip);
      int part;
      std::vector<int> ipParts;
      while (ipStream >> part) {
        ipParts.push_back(part);
        if (ipStream.get() == '/') break;
      }
      if (!ipParts.empty()) {
        return ipParts.back();
      }
    }
  }
  return -1;
}

void TeleopCmdNode::status_timer_callback(void)
{
  FILE * fp;
  char buffer[1024];
  fp = popen("ifconfig | grep ap0", "r");
  if (fp == NULL) {
    perror("popen");
    exit(EXIT_FAILURE);
  }
  this->transfer_data.hotspot_mode = 0;
  while (fgets(buffer, sizeof(buffer), fp) != NULL) {
    this->transfer_data.hotspot_mode = 1;
    break;
  }
  pclose(fp);
  std::string ipAddrOutput = exec("ip addr show wlan0");
  this->transfer_data.ip_end = getWlan0IpLastByte(ipAddrOutput);
}

uint32_t TeleopCmdNode::date_to_uint32(const char * date_str)
{
  char year[5], month[4], day[3];
  sscanf(date_str, "%3s %2s %4s", month, day, year);

  // 将月份转换为数字（1-12）
  int mon_num = 0;
  if (strcmp(month, "Jan") == 0)
    mon_num = 1;
  else if (strcmp(month, "Feb") == 0)
    mon_num = 2;
  else if (strcmp(month, "Mar") == 0)
    mon_num = 3;
  else if (strcmp(month, "Apr") == 0)
    mon_num = 4;
  else if (strcmp(month, "May") == 0)
    mon_num = 5;
  else if (strcmp(month, "Jun") == 0)
    mon_num = 6;
  else if (strcmp(month, "Jul") == 0)
    mon_num = 7;
  else if (strcmp(month, "Aug") == 0)
    mon_num = 8;
  else if (strcmp(month, "Sep") == 0)
    mon_num = 9;
  else if (strcmp(month, "Oct") == 0)
    mon_num = 10;
  else if (strcmp(month, "Nov") == 0)
    mon_num = 11;
  else if (strcmp(month, "Dec") == 0)
    mon_num = 12;

  // 将年份、月份和日期转换为整数并组合成YYYYMMDD格式
  uint32_t date_num = (atoi(year) * 10000) + (mon_num * 100) + atoi(day);
  return date_num;
}

void TeleopCmdNode::mode_switch_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  int axis_value = static_cast<int>(joy_msg->axes[8]);
  if (this->last_key == 0 && axis_value) {
    switch (axis_value) {
      case TITA_STOP_OBST_MODE: {
        // RCLCPP_INFO(this->get_logger(), "In stop mode");
        if (this->transfer_data.stop_obst_mode) {
          if (waitForService(obstacle_param_client_, "obstacle_start")) {
            this->transfer_data.stop_obst_mode = 0;
            set_parameter(obstacle_param_client_, "obstacle_start", false);
          }
        } else {
          if (waitForService(obstacle_param_client_, "obstacle_start")) {
            this->transfer_data.stop_obst_mode = 1;
            set_parameter(obstacle_param_client_, "obstacle_start", true);
          }
        }
      } break;
      case TITA_BYPASS_OBST_MODE: {
        if (this->transfer_data.bypass_obst_mode) {
          int result = system("wifi-app apoff");
          if (result == -1) perror("system");
        } else {
          int result = system("wifi-app ap");
          if (result == -1) perror("system");
        }
      } break;
      case TITA_USE_SDK_MODE: {
        if (this->transfer_data.use_sdk_mode) {
          if (waitForService(sdk_param_client_, "use_sdk")) {
            this->transfer_data.use_sdk_mode = 0;
            set_parameter(sdk_param_client_, "use_sdk", false);
          }
        } else {
          if (waitForService(sdk_param_client_, "use_sdk")) {
            this->transfer_data.use_sdk_mode = 1;
            set_parameter(sdk_param_client_, "use_sdk", true);
          }
        }
      } break;
      case TITA_SPEED_LIMIT_MODE: {
        if (this->transfer_data.speed_limit_mode) {
          int result = system("wifi-app apoff");
          if (result == -1) perror("system");
        } else {
          int result = system("wifi-app ap");
          if (result == -1) perror("system");
        }
      } break;
      case TITA_IMAGE_TRAN_MODE: {
        if (this->transfer_data.auto_mapping_mode) {
          int result = system("wifi-app apoff");
          if (result == -1) perror("system");
        } else {
          int result = system("wifi-app ap");
          if (result == -1) perror("system");
        }
      } break;
      case TITA_HOTSPOT_MODE: {
        if (this->transfer_data.hotspot_mode) {
          int result = system("wifi-app apoff");
          if (result == -1) perror("system");
        } else {
          int result = system("wifi-app ap");
          if (result == -1) perror("system");
        }
      } break;
      case TITA_WIFI_CONNECT_MODE: {
        std::stringstream ss;
        for (int i = 0; i < 33; ++i) {
          if (this->wifi_data[i] >= 0 && this->wifi_data[i] <= 127) {
            ss << static_cast<char>(this->wifi_data[i]);
          } else {
            continue;
          }
        }
        std::string result = ss.str();
        std::cout << "合成的字符串为: " << result << std::endl;
        this->wifi_key_offset = 0;
        std::stringstream ss1;
        ss1 << "nmcli device wifi list|wifi-app connect " << result;
        std::string command = ss1.str();
        std::cout << "Executing command: " << command << std::endl;
        int status = system(command.c_str());
        if (status != 0) {
          std::cerr << "Error executing command: " << command << std::endl;
        }
      } break;

      default:
        break;
    }

    if (axis_value > TITA_WIFI_CONNECT_MODE && axis_value < -1) {
      this->wifi_data[this->wifi_key_offset] = axis_value + 131;
      this->wifi_key_offset++;
      printf("%d\n", axis_value);
    }
  }
  this->transfer_data.version = date_to_uint32(__DATE__);
  this->transfer_data.error_code = 0x07;
  this->transfer_data.bat_left = static_cast<int>(lbattery_info.percentage);
  this->transfer_data.bat_right = static_cast<int>(rbattery_info.percentage);
  // RCLCPP_INFO(this->get_logger(), "Send Data %d", this->transfer_data.stop_obst_mode);
  crsf.setTransferBackData(reinterpret_cast<uint8_t *>(&transfer_data), 0, sizeof(transfer_data));
  this->last_key = axis_value;
}

void TeleopCmdNode::ModeReadcallback(const uint8_t data[])
{
  switch (data[0]) {
    case CRSF_CUSTOMER_CMD_DISCONNECT:
      printf("link disconnect!!!\n");
      break;
    case CRSF_CUSTOMER_CMD_IS_BINGDING:
      printf("elrs is in binding!!!\n");
      break;
    case CRSF_CUSTOMER_CMD_UART_LINKED:
      printf("elrs uart connect success!!!\n");
      break;
    default:
      break;
  }
}

void TeleopCmdNode::DataReadCallBack(const uint16_t channels[])
{
#if (0)
  printf(
    "CH1: %d\tCH2: %d\tCH3: %d\tCH4: %d\tCH5: %d\tCH6: %d\tCH7: %d\tCH8: %d\tCH9: %d\t\n",
    channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], channels[6],
    channels[7], channels[8]);
  printf("\n");
#endif
  auto joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
  joy_msg->header.stamp = this->now();
  joy_msg->header.frame_id = "joy";
  float normalized1 = -1.0 * normalizeValue(channels[0], 172, 1638);
  float normalized2 = -1.0 * normalizeValue(channels[1], 172, 1638);
  float normalized3 = -1.0 * normalizeValue(channels[2], 172, 1638);
  float normalized4 = -1.0 * normalizeValue(channels[3], 172, 1638);
  if (std::abs(normalized1) < _AxesBias) normalized1 = 0.0;
  if (std::abs(normalized2) < _AxesBias) normalized2 = 0.0;
  if (std::abs(normalized3) < _AxesBias) normalized3 = 0.0;
  if (std::abs(normalized4) < _AxesBias) normalized4 = 0.0;
  joy_msg->axes.push_back(normalized1);
  joy_msg->axes.push_back(normalized2);
  joy_msg->axes.push_back(normalized3);
  joy_msg->axes.push_back(normalized4);
  joy_msg->axes.push_back(-(static_cast<int>(channels[4]) / 800 - 1));
  joy_msg->axes.push_back(-(static_cast<int>(channels[5]) / 900 - 1));
  joy_msg->axes.push_back(-(static_cast<int>(channels[6]) / 800 - 1));
  joy_msg->axes.push_back(-(static_cast<int>(channels[7]) / 900 - 1));
  joy_msg->axes.push_back((static_cast<int>(channels[8] - 988) / 4));
  elrs_publisher_->publish(std::move(joy_msg));
}

float TeleopCmdNode::normalizeValue(float originalValue, float minOriginal, float rangeOriginal)
{
  return ((originalValue - minOriginal) / rangeOriginal) * 2 - 1;
}

bool TeleopCmdNode::waitForService(
  const rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client,
  const std::string & node_name)
{
  int wait_count = 0;
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    wait_count += 1;
    if (wait_count > 3) {
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for %s parameter service...", node_name.c_str());
  }
  return true;
}

void TeleopCmdNode::set_parameter(
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client,
  std::string param_name, bool mark)
{
  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rclcpp::Parameter parameter(param_name, mark);
  request->parameters.push_back(parameter.to_parameter_msg());
  auto result = param_client->async_send_request(request);
}
