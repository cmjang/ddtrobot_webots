#include "fsm/FSMState_Passive.h"
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

FSMState_Passive::FSMState_Passive(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::PASSIVE, "passive")
{
}

void FSMState_Passive::enter() { _data->state_command->firstRun = true; }

void FSMState_Passive::run()
{
  //   std::make_shared<DesiredStateCommand::RemoteControlData>());
  _data->state_command->clear();
  _data->low_cmd->zero();

  // if(!_data->params_->ee_name_.empty()){
  //   Eigen::VectorXd kp_joint, kd_joint;
  //   kp_joint.setZero(6);
  //   kd_joint.setZero(6);
  //   kp_joint << 100, 100, 10, 1, 1, 1;
  //   // kd_joint << 1, 1, 1, 0.1, 0.1, 0.1;
  //   _data->low_cmd_->tau_cmd.tail(6) = kp_joint.cwiseProduct(-_data->low_state_->q.tail(6)) + kd_joint.cwiseProduct(-_data->low_state_->dq.tail(6));
  //   PRINT_MAT(_data->low_state_->dq.tail(6));
  // }
  // _data->_stateEstimator->run();
}

void FSMState_Passive::exit() {}

FSMStateName FSMState_Passive::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  if (fsm_name_pre_ != _data->state_command->desire_data_->fsm_state_name) {
    switch (_data->state_command->desire_data_->fsm_state_name) {
      case FSMStateName::RECOVERY_STAND:
        this->_nextStateName = FSMStateName::RECOVERY_STAND;
        break;

      case FSMStateName::PASSIVE:  // normal c
        break;
      case FSMStateName::JOINT_PD:  // normal c
        this->_nextStateName = FSMStateName::JOINT_PD;
        break;
      case FSMStateName::RL:
        this->_nextStateName = FSMStateName::RL;
        break;

      default:
        break;
    }
  }
  fsm_name_pre_ = _data->state_command->desire_data_->fsm_state_name;
  return this->_nextStateName;
}
