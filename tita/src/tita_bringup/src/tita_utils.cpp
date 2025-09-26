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

#include "tita_utils/tita_utils.hpp"

namespace tita_utils
{

Quaternion rollToQuaternion(double pitch)
{
  Quaternion q;
  q.w = std::cos(pitch / 2.0);
  q.x = std::sin(pitch / 2.0);
  q.y = 0.0;
  q.z = 0.0;
  return q;
}

Quaternion pitchToQuaternion(double yaw)
{
  Quaternion q;
  q.w = std::cos(yaw / 2.0);
  q.x = 0.0;
  q.y = std::sin(yaw / 2.0);
  q.z = 0.0;
  return q;
}

Quaternion yawToQuaternion(double roll)
{
  Quaternion q;
  q.w = std::cos(roll / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(roll / 2.0);
  return q;
}

Quaternion multiplyQuaternions(const Quaternion & q1, const Quaternion & q2)
{
  Quaternion q;
  q.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  q.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  q.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
  q.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
  return q;
}

std::string actionToString(Action action)
{
  auto it = actionToStringMap.find(action);
  if (it != actionToStringMap.end()) {
    return it->second;
  }
  return "unknown";
}

Action stringToAction(const std::string & actionString)
{
  auto it = stringToActionMap.find(actionString);
  if (it != stringToActionMap.end()) {
    return it->second;
  }
  return Action::Unknown;
}

void utils::pidInit(float Kp, float Ki, float Kd, float outputMin, float outputMax)
{
  pid.Kp = Kp;
  pid.Ki = Ki;
  pid.Kd = Kd;
  pid.outputMin = outputMin;
  pid.outputMax = outputMax;
}

void utils::clean_param() { previousError = 0.0; }

float utils::pidCompute(float currentValue, float target_point, float dt)
{
  float error = target_point - currentValue;
  integral += error * dt;
  float derivative = (error - previousError) / dt;

  float output = pid.Kp * error + pid.Ki * integral + pid.Kd * derivative;

  if (output > pid.outputMax) {
    output = pid.outputMax;
  } else if (output < pid.outputMin) {
    output = pid.outputMin;
  }
  previousError = error;
  return output;
}

}  // namespace tita_utils
