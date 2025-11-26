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

#ifndef TELEOP_COMMAND__BUTTON_FUNCTION_HPP_
#define TELEOP_COMMAND__BUTTON_FUNCTION_HPP_

#include <iostream>
#include <string>
#include <tuple>

/// \brief  check button param. Get button index.
/// \param  string: button parameter.
/// \returns return a tuple -1: wrong; 0: axes; 1:button.
std::tuple<int, int> checkAndExtract(const std::string & str);

/// \brief  map axes value to button value;
/// \param  string: axes parameter.
/// \returns double: button value.
double mapToButtonValue(double x);

#endif  // TELEOP_COMMAND__BUTTON_FUNCTION_HPP_
