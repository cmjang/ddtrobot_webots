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

#ifndef ACTIVE_COMMAND__STATE_MACHINE_HPP_
#define ACTIVE_COMMAND__STATE_MACHINE_HPP_

#include <functional>
#include <iostream>
#include <map>
#include <string>

namespace command
{
enum class State {
  IDLE,
  EMERGENCY_STOP,
  CHARGE,
  JUMP,
  BALANCE_STAND,
  TRANSFORM_DOWN,
  TRANSFORM_UP,
  SUSPENDING,
  CRASH
};

enum class Event {
  EVENT_IDLE_TRANSUP,
  EVENT_TRANSUP_BALANCE,
  EVENT_TRANSUP_CRASH,
  EVENT_STOP_IDLE,
  EVENT_TRANSDOWN_IDLE,
  EVENT_BALANCE_TRANSDOWN,
  EVENT_BALANCE_CHARGE,
  EVENT_BALANCE_CRASH,
  EVENT_BALANCE_STOP,
  EVENT_BALANCE_SUSUPEND,
  EVENT_CRASH_IDLE,
  EVENT_CHARGE_JUMP,
  EVENT_JUMP_BALANCE,
  EVENT_JUMP_STOP,
  EVENT_SUSPEND_STOP,
  EVENT_SUSPEND_CRASH,
};

class StateMachine
{
public:
  StateMachine();
  int handle_event(Event event);
  void set_state(State state);
  State string2state(std::string fsm_mode);
  std::string get_string_state();

private:
  State current_state_;
  std::string str_state_;
  std::map<State, std::map<Event, State>> transitions_;
  std::map<State, std::function<void()>> state_actions_;
};

}  // namespace command
#endif  // ACTIVE_COMMAND__STATE_MACHINE_HPP_
