/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_JointPD.h"
// #include <Utilities/Utilities_print.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSMState_JointPD::FSMState_JointPD(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::JOINT_PD, "joint_pd")
{
  // Do nothing
  // Set the pre controls safety checks
  // this->checkSafeOrientation = false;

  // Post control safety checks
  // this->checkPDesFoot = false;
  // this->checkForceFeedForward = false;
  size_t dof = _data->params->dof_chassis;
  initial_jpos.setZero(dof);
}

void FSMState_JointPD::enter()
{
  // Default is to not transition
  this->_nextStateName = this->_stateName;
  initial_jpos = this->_data->low_state->q;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */

void FSMState_JointPD::run()
{
  _data->low_cmd->zero();
  Eigen::Map<DVec<scalar_t>> kp_joint(
    _data->params->joint_kp.data(), _data->params->joint_kp.size()),
    kd_joint(_data->params->joint_kd.data(), _data->params->joint_kd.size());

  DVec<scalar_t> initial_djpos(initial_jpos.size());
  initial_djpos.setZero();
  _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(initial_jpos - _data->low_state->q) +
                            kd_joint.cwiseProduct(initial_djpos - _data->low_state->dq);
  for (Eigen::Index i(0); i < initial_jpos.size(); ++i) {
    bound(_data->low_cmd->tau_cmd(i), _data->params->torque_limit[i]);
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */

FSMStateName FSMState_JointPD::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (_data->state_command->desire_data_->fsm_state_name) {
    case FSMStateName::RECOVERY_STAND:
      break;

    case FSMStateName::PASSIVE:  // normal c
      this->_nextStateName = FSMStateName::PASSIVE;
      break;
    default:
      break;
      // std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
      //           << K_RECOVERY_STAND << " to "
      //           << this->_data->controlParameters->control_mode << std::endl;
  }
  // Get the next state
  return this->_nextStateName;
}

/**
 * Cleans up the state information on exiting the state.
 */

void FSMState_JointPD::exit()
{
  // Nothing to clean up when exiting
}
