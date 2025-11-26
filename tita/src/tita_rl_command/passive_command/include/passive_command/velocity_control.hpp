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

#ifndef PASSIVE_COMMAND__VELOCITY_CONTROL_HPP_
#define PASSIVE_COMMAND__VELOCITY_CONTROL_HPP_

#include <cmath>

/// \brief  add A and B. This is a note example。
/// \param  double number A.
/// \param  double number B.
/// \returns return a double numble.
double add_function(double A, double B);

/// \brief map obstacle distance to max speed.
/// \returns return g(x)=0.921359 log(10,x-0.25)+0.590218
double velocity_remap(double distance);

struct PASSIVE_CMD
{
  double max_turn;
  double max_speed;
};

#endif  // PASSIVE_COMMAND__VELOCITY_CONTROL_HPP_
