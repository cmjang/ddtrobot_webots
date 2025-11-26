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
#ifndef TITA_ROBOT__DEF_CAN_H_
#define TITA_ROBOT__DEF_CAN_H_

#include "protocol/can_utils.hpp"

namespace can_device
{
#define PACKED __attribute__((__packed__))

#define GRAVITY 9.81f

#define CAN_NAME "can0"

#define CAN_ID_STATE 0x102
#define CAN_ID_LOCOMOTION_STATE 0x103

#define CAN_ID_IMU1 0x118
#define CAN_ID_IMU2 0x119

#define CAN_ID_MOTOR_IN 0x108
#define CAN_ID_SEND_MOTORS 0x120
#define CAN_ID_CHANNEL_INPUT 0x12D
#define CAN_ID_RPC_REQUEST 0x170

struct api_motor_in_t
{
  uint32_t timestamp;
  float position;
  float velocity;
  float torque;
} PACKED;

/* 44 bytes */
struct api_imu_data_t
{
  uint32_t timestamp;
  float accl[3];
  float gyro[3];
  float quaternion[4];  // x y z w
  float temperature;
} PACKED;

struct api_motor_status_t
{
  uint32_t timestamp;
  uint16_t key;
  uint8_t value[8];
} PACKED;

struct api_locomotion_status_t
{
  uint32_t timestamp;
  uint16_t key;
  union {
    uint8_t u8;
    uint8_t u8x4[4];
    uint16_t u16;
    uint16_t u16x4[4];
    uint32_t u32;
    uint32_t u32x4[4];
    float f32;
    float f32x4[4];
    float f32x9[9];
  } value;
} PACKED;

enum locomotion_status_key_t {
  ROBOT_MODE = 0x1001,
  JUMP_STATUS = 0x1002,
};

enum robot_status_t {
  DIE = 0x00,
  INIT = 0x01,
  TRANSFORM_UP = 0x02,
  STAND = 0x03,
  TRANSFORM_DOWN = 0x04,
  CRASH = 0x05,
  SUSPENDING = 0x06,
  JUMP = 0x07,
};

enum jump_status_t {
  LIFT = 0x00,
  RETRACT = 0x01,
  EXTEND = 0x02,
  FINISH = 0x03,
  DISCHARGE = 0x04,
  CHARGE = 0x05,
  OFF = 0x06,
};

enum api_rpc_key_t {
  RPC_UNDEFINED = 0x000,
  GET_MODEL_INFO = 0x100,
  GET_SERIAL_NUMBER = 0x101,
  SET_READY_NEXT = 0x200,
  SET_BOARDCAST = 0x201,
  SET_INPUT_MODE = 0x221,
  SET_STAND_MODE = 0x222,
  SET_HEAD_MODE = 0x223,
  SET_JUMP = 0x231,
  SET_MOTOR_ZERO = 0x280,
};

enum ready_next_t {
  READY_WAITING = 0x00U,
  AUTO_LOCOMOTION = 0x01U,
  FORCE_LOCOMOTION = 0x02U,
  FORCE_DIRECT = 0x03U,
  MOTOR_ZERO_CAL = 0x04U,
  BOOT_RECOVERY_MODE = 0x05U,
  ESTOP_MODE = 0x06U,
};

struct api_channel_input_t
{
  uint32_t timestamp;
  float forward;
  float yaw;
  float pitch;
  float roll;
  float height;
  float split;
  float tilt;
  float forward_accel;
  float yaw_accel;
} PACKED;

struct api_rpc_response_t
{
  uint32_t timestamp;
  uint16_t key;
  uint32_t value;
} PACKED;

struct api_motor_out_t
{
  uint32_t timestamp;
  float position;
  float kp;
  float velocity;
  float kd;
  float torque;
} PACKED;

inline uint32_t get_current_time()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return std::move(tv.tv_sec * 1000000 + tv.tv_usec);
}

}  // namespace can_device
#endif  // TITA_ROBOT__DEF_CAN_H_