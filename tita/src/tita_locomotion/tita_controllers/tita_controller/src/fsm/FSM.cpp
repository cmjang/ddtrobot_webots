#include "fsm/FSM.h"

#include <iostream>

FSM::FSM(std::shared_ptr<ControlFSMData> data) : _data(data)
{
  _stateList.invalid = nullptr;
  _stateList.passive = new FSMState_Passive(_data);
  // _stateList.balanceStand = new FSMState_BalanceStand(_data);
  _stateList.recoveryStand = new FSMState_RecoveryStand(_data);
  _stateList.jointPD = new FSMState_JointPD(_data);
  _stateList.transformDown = new FSMState_TransformDown(_data);
  _stateList.rl = new FSMState_RL(_data);
  initialize();
}

FSM::~FSM() { _stateList.deletePtr(); }

void FSM::initialize()
{
  count = 0;
  _currentState = _stateList.passive;
  _currentState->enter();
  _nextState = _currentState;
  _mode = FSMMode::NORMAL;
  std::cout << "mode: " << _currentState->_stateNameStr.c_str() << std::endl;
}

void FSM::run()
{
  if (!checkSafty()) {
    _currentState = _stateList.passive;
    _currentState->run();
    return;
  }

  if (_mode == FSMMode::NORMAL) {
    _currentState->run();
    _nextStateName = _currentState->checkTransition();
    if (_nextStateName != _currentState->_stateName) {
      _mode = FSMMode::CHANGE;
      _nextState = getNextState(_nextStateName);
    }
  } else if (_mode == FSMMode::CHANGE) {
    _currentState->exit();
    _currentState = _nextState;
    _currentState->enter();
    _mode = FSMMode::NORMAL;
    _currentState->run();
    std::cout << "mode: " << _currentState->_stateNameStr.c_str() << std::endl;
  }
  // limit torque out put
  for (Eigen::Index i = 0; i < _data->low_cmd->tau_cmd.size(); i++) {
    bound(_data->low_cmd->tau_cmd[i], _data->params->torque_limit[i]);
  }

  count++;
}

FSMState * FSM::getNextState(FSMStateName stateName)
{
  switch (stateName) {
    case FSMStateName::INVALID:
      return _stateList.invalid;
      break;
    case FSMStateName::PASSIVE:
      return _stateList.passive;
      break;
    // case FSMStateName::BALANCE_STAND:
      // return _stateList.balanceStand;
      // break;
    case FSMStateName::RECOVERY_STAND:
      return _stateList.recoveryStand;
      break;
    case FSMStateName::JOINT_PD:
      return _stateList.jointPD;
      break;
    case FSMStateName::TRANSFORM_DOWN:
      return _stateList.transformDown;
      break;
    case FSMStateName::RL:
      return _stateList.rl;
      break;
    default:
      return _stateList.passive;
      break;
  }
}

bool FSM::checkSafty()
{
  bool isSafe = true;
  // if (_data->state_estimator->getResult().rBody(2, 2) < 0.5) isSafe = false;
  // if (_currentState->_stateName == FSMStateName::BALANCE_STAND) {
  //   if (std::abs(_data->wheel_legged_data->com_position_relative(point::X)) > 0.15) isSafe = false;
  // }
  return isSafe;
}