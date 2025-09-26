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

#ifndef TITA_UTILS__TITA_UTILS_HPP_
#define TITA_UTILS__TITA_UTILS_HPP_

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

namespace tita_utils
{

struct Quaternion
{
  double w, x, y, z;
};

enum class Action {
  Space,
  StandHigh,
  StandMid,
  StandLow,
  SpeedHigh,
  SpeedMid,
  SpeedLow,
  JumpUp,
  JumpDown,
  Die,
  Unknown
};

const std::unordered_map<Action, std::string> actionToStringMap = {
  {Action::Space, ""},
  {Action::StandHigh, "stand_high"},
  {Action::StandMid, "stand_mid"},
  {Action::StandLow, "stand_low"},
  {Action::SpeedHigh, "speed_high"},
  {Action::SpeedMid, "speed_mid"},
  {Action::SpeedLow, "speed_low"},
  {Action::JumpUp, "jump_up"},
  {Action::JumpDown, "jump_down"},
  {Action::Die, "die"},
  {Action::Unknown, "unknown"}};

std::unordered_map<std::string, Action> stringToActionMap;

Action stringToAction(const std::string & actionString);

std::string actionToString(Action action);

Quaternion rollToQuaternion(double roll);

Quaternion pitchToQuaternion(double pitch);

Quaternion yawToQuaternion(double yaw);

Quaternion multiplyQuaternions(const Quaternion & q1, const Quaternion & q2);

struct PIDController
{
  float Kp;
  float Ki;
  float Kd;
  float outputMin;
  float outputMax;
};

class utils
{
private:
  float integral = 0.0;
  float previousError = 0.0;

public:
  utils(/* args */) {}
  ~utils() {}
  PIDController pid;
  void pidInit(float Kp, float Ki, float Kd, float outputMin, float outputMax);
  float pidCompute(float currentValue, float target_point, float dt);
  void clean_param();
};

}  // namespace tita_utils

#endif  // TITA_UTILS__TITA_UTILS_HPP_
