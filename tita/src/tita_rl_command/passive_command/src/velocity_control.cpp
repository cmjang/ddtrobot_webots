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

#include "passive_command/velocity_control.hpp"

#include <iostream>
double add_function(double A, double B) { return A + B; }

double velocity_remap(double distance)
{
  // double result = 0.6 * (std::log(distance - 0.3) / std::log(5)) + 0.933;
  double result = 0.921359 * std::log10(distance - 0.25) + 0.590218;
  if (result < 0.0) result = 0.0;
  return result;
}
