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

#ifndef MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_
#define MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "tita_robot/def_can.h"

namespace can_device
{

class MotorsCanSendApi
{
public:
  MotorsCanSendApi(size_t size) { (void)size; }
  ~MotorsCanSendApi() = default;
  bool send_motors_can(std::vector<api_motor_out_t> motors);
  bool send_command_can_channel_input(api_channel_input_t data);
  bool send_command_can_rpc_request(api_rpc_response_t data);

private:
#define MIN_TIME_OUT_US 1'000L      // 1ms
#define MAX_TIME_OUT_US 3'000'000L  // 3s
  std::string can_interface = "can0";
  std::string can_name = "motors_can_send";
  int64_t timeout_us = MAX_TIME_OUT_US;
  uint8_t can_id_offset = 0x00U;
  bool can_extended_frame = false;
  bool can_fd_mode = true;

  std::shared_ptr<can_device::socket_can::CanDev> can_send_api_ =
    std::make_shared<can_device::socket_can::CanDev>(
      can_interface, can_name, can_extended_frame, can_fd_mode, timeout_us, can_id_offset);
};
}  // namespace can_device

#endif  // MOTORS_CAN__MOTORS_CAN_SEND_API_HPP_
