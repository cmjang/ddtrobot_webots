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

#include "active_command/state_machine.hpp"

namespace command
{

StateMachine::StateMachine()
{
  current_state_ = State::IDLE;
  str_state_ = "idle";
  transitions_ = {
    {State::IDLE, {{Event::EVENT_IDLE_TRANSUP, State::TRANSFORM_UP}}},
    {State::EMERGENCY_STOP, {{Event::EVENT_STOP_IDLE, State::IDLE}}},
    {State::CHARGE, {{Event::EVENT_CHARGE_JUMP, State::JUMP}}},
    {State::JUMP,
     {{Event::EVENT_JUMP_BALANCE, State::BALANCE_STAND},
      {Event::EVENT_JUMP_STOP, State::EMERGENCY_STOP}}},
    {State::BALANCE_STAND,
     {{Event::EVENT_BALANCE_CRASH, State::CRASH},
      {Event::EVENT_BALANCE_CHARGE, State::CHARGE},
      {Event::EVENT_BALANCE_SUSUPEND, State::SUSPENDING},
      {Event::EVENT_BALANCE_STOP, State::EMERGENCY_STOP},
      {Event::EVENT_BALANCE_TRANSDOWN, State::TRANSFORM_DOWN}}},
    {State::TRANSFORM_DOWN, {{Event::EVENT_TRANSDOWN_IDLE, State::IDLE}}},
    {State::TRANSFORM_UP,
     {{Event::EVENT_TRANSUP_BALANCE, State::BALANCE_STAND},
      {Event::EVENT_TRANSUP_CRASH, State::CRASH}}},
    {State::SUSPENDING,
     {{Event::EVENT_SUSPEND_CRASH, State::CRASH},
      {Event::EVENT_SUSPEND_STOP, State::EMERGENCY_STOP}}},
    {State::CRASH, {{Event::EVENT_CRASH_IDLE, State::IDLE}}},
  };
  state_actions_ = {
    {State::IDLE, [this]() { str_state_ = "idle"; }},
    {State::EMERGENCY_STOP, [this]() { str_state_ = "stop"; }},
    {State::CHARGE, [this]() { str_state_ = "charge"; }},
    {State::JUMP, [this]() { str_state_ = "jump"; }},
    {State::BALANCE_STAND, [this]() { str_state_ = "balance"; }},
    {State::TRANSFORM_DOWN, [this]() { str_state_ = "transform_down"; }},
    {State::TRANSFORM_UP, [this]() { str_state_ = "transform_up"; }},
    {State::SUSPENDING, [this]() { str_state_ = "suspending"; }},
    {State::CRASH, [this]() { str_state_ = "crash"; }}};
}

int StateMachine::handle_event(Event event)
{
  auto it = transitions_.find(current_state_);
  if (it != transitions_.end()) {
    auto event_it = it->second.find(event);
    if (event_it != it->second.end()) {
      current_state_ = event_it->second;
      state_actions_[current_state_]();
      return 0;
    } else {
      // std::cout << "Event not valid for current state.\n";
      return 1;
    }
  } else {
    // std::cout << "Current state has no transitions.\n";
    return 2;
  }
}

void StateMachine::set_state(State state)
{
  current_state_ = state;
  state_actions_[current_state_]();
}

State StateMachine::string2state(std::string fsm_mode)
{
  State user_state = current_state_;
  if (fsm_mode == "idle")
    user_state = State::IDLE;
  else if (fsm_mode == "stop")
    user_state = State::EMERGENCY_STOP;
  else if (fsm_mode == "charge")
    user_state = State::CHARGE;
  else if (fsm_mode == "jump")
    user_state = State::JUMP;
  else if (fsm_mode == "balance")
    user_state = State::BALANCE_STAND;
  else if (fsm_mode == "transform_down")
    user_state = State::TRANSFORM_DOWN;
  else if (fsm_mode == "transform_up")
    user_state = State::TRANSFORM_UP;
  else if (fsm_mode == "suspending")
    user_state = State::SUSPENDING;
  else if (fsm_mode == "crash")
    user_state = State::CRASH;
  return user_state;
}

std::string StateMachine::get_string_state() { return str_state_; }

}  // namespace command
