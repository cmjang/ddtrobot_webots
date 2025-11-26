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

#ifndef ACTIVE_COMMAND__USER_BTN_FUNCTION_HPP_
#define ACTIVE_COMMAND__USER_BTN_FUNCTION_HPP_

/// \brief  map x from [-1,1] to [range_min,range_max]
/// \param  double range_min.
/// \param  double range_max.
/// \returns return map result.
double mapToRange(double x, double range_min, double range_max);

#endif  // ACTIVE_COMMAND__USER_BTN_FUNCTION_HPP_
