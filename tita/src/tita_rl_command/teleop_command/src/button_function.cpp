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

#include "teleop_command/button_function.hpp"

std::tuple<int, int> checkAndExtract(const std::string & str)
{
  if (str.empty()) {
    return std::make_tuple(-1, -1);
  }
  try {
    int number = std::stoi(str.substr(1));
    if (str[0] == 'a') {
      return std::make_tuple(0, number);
    } else if (str[0] == 'b') {
      return std::make_tuple(1, number);
    }
  } catch (const std::invalid_argument & e) {
    return std::make_tuple(0, 0);
  } catch (const std::out_of_range & e) {
    return std::make_tuple(0, 0);
  }
  return std::make_tuple(0, 0);
}

double mapToButtonValue(double x)
{
  if (x < -1.0) x = -1.0;
  if (x > 1.0) x = 1.0;
  if (x < -1.0 / 3.0) {
    return -1.0;
  } else if (x < 1.0 / 3.0) {
    return 0.0;
  } else {
    return 1.0;
  }
}
