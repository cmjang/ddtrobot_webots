/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_RecoveryStand.h"
// #include <Utilities/Utilities_print.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
FSMState_RecoveryStand::FSMState_RecoveryStand(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::RECOVERY_STAND, "recovery_stand")
{
  // Do nothing
  // Set the pre controls safety checks
  // this->checkSafeOrientation = false;

  // Post control safety checks
  // this->checkPDesFoot = false;
  // this->checkForceFeedForward = false;
  size_t dof = _data->params->dof_chassis;

  fold_jpos.setZero(dof);
  stand_jpos.setZero(dof);
  rolling_jpos.setZero(dof);
  initial_jpos.setZero(dof);
  f_ff.setZero(dof);
  if (dof == 8) {
    // goal configuration
    // Folding
    fold_jpos << 0.75f, 1.20f, -2.7f, 0.0f, -0.75f, 1.2f, -2.7f, 0.0f;
    // Stand Up
    stand_jpos << -0.05f, 0.9f, -2.6f, 0.0f, 0.05f, 0.90f, -2.6f, 0.0f;
    // Rolling
    rolling_jpos << -0.32f, -1.5f, -2.7f, 0.0f, -0.32f, -1.5f, -2.7f, 0.0f;
    f_ff << -20.f, 0.f, -25.f, 0.0f, 20.f, 0.f, -25.f, 0.0f;
  } else {
    // goal configuration
    // Folding
    fold_jpos << 1.2f, -2.6f, 0.0f, 1.2f, -2.6f, 0.0f;
    // Stand Up
    stand_jpos << 1.2f, -2.4f, 0.0f, 1.2f, -2.4f, 0.0f;
    // Rolling
    rolling_jpos << 1.5f, -1.6f, 2.77f, 1.3f, -3.1f, 2.77f;
    f_ff << 0.f, -25.f, 0.f, 0.f, -25.f, 0.f;
  }
}

void FSMState_RecoveryStand::enter()
{
  // Default is to not transition
  this->_nextStateName = this->_stateName;
  fold_ramp_iter = timer_fold * _data->params->update_rate;
  standup_ramp_iter = timer_standup * _data->params->update_rate;
  rollover_ramp_iter = timer_rollover * _data->params->update_rate;

  // // Reset the transition data
  // this->transitionData.zero();

  // // Reset iteration counter
  iter = 0;
  _state_iter = 0;

  // initial configuration, position
  initial_jpos = this->_data->low_state->q;
  // initial_jpos(3) = initial_jpos(7) = 0;
  fold_jpos(3) = initial_jpos(3);
  fold_jpos(7) = initial_jpos(7);

  fold_ramp_iter = fold_ramp_iter * (initial_jpos - fold_jpos).cwiseAbs().maxCoeff();  //
  // PRINT_MAT(initial_jpos - fold_jpos);
  // printf("fold_ramp_iter:%d\n", fold_ramp_iter);
  // printf("maxCoeff      :%f\n", (initial_jpos - fold_jpos).cwiseAbs().maxCoeff());

  // scalar_t body_height = this->_data->_stateEstimator->getResult().position[2];

  _flag = FoldLegs;
  if (!_UpsideDown()) {
    // Proper orientation
    // if (  (0.2 < body_height) && (body_height < 0.45) ){
    //   printf("[Recovery Balance] body height is %f; Stand Up \n", body_height);
    //   _flag = StandUp;
    // }else{
    //   printf("[Recovery Balance] body height is %f; Folding legs \n", body_height);
    // }
  } else {
    printf("[Recovery Balance] UpsideDown (%d) \n", _UpsideDown());
    _flag = RollOver;
  }
  _motion_start_iter = 0;
}

bool FSMState_RecoveryStand::_UpsideDown()
{
  //pretty_print(this->_data->_stateEstimator->getResult().rBody, std::cout, "Rot");
  //if(this->_data->_stateEstimator->getResult().aBody[2] < 0){
  if (this->_data->state_estimator->getResult().rBody(2, 2) < 0) {
    return true;
  }
  return false;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */

void FSMState_RecoveryStand::run()
{
  // printf("flag: %d, _state_iter: %d, _motion_start_iter: %d\n", _flag, _state_iter, _motion_start_iter);
  _data->low_cmd->zero();
  switch (_flag) {
    case StandUp:
      _StandUp(_state_iter - _motion_start_iter);
      // std::cout << "Stand Up" << std::endl;
      break;
    case FoldLegs:
      _FoldLegs(_state_iter - _motion_start_iter);
      // std::cout << "Fold Legs" << std::endl;
      break;
    case RollOver:
      _RollOver(_state_iter - _motion_start_iter);
      // std::cout << "Roll Over" << std::endl;
      break;
    default:
      std::cout << "ERROR: Recovery Stand State not defined" << std::endl;
      break;
  }
  ++_state_iter;
}

DVec<scalar_t> FSMState_RecoveryStand::_SetJPosInterPts(
  const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini, const DVec<scalar_t> & fin)
{
  if (ini.size() != fin.size()) {
    std::cerr << "[FSMState_RecoveryStand] ERROR: ini and fin must have the same size" << std::endl;
  }

  float a(0.f);
  float b(1.f);

  // if we're done interpolating
  if (curr_iter <= max_iter) {
    b = (float)curr_iter / (float)max_iter;
    a = 1.f - b;
  }

  // compute setpoints
  DVec<scalar_t> inter_pos = a * ini + b * fin;
  return inter_pos;
}

void FSMState_RecoveryStand::_RollOver(const int & curr_iter)
{
  // _SetJPosInterPts(curr_iter, rollover_ramp_iter, initial_jpos, rolling_jpos);

  // if(curr_iter > rollover_ramp_iter + rollover_settle_iter){
  //   _flag = FoldLegs;
  //   initial_jpos = rolling_jpos;
  //   _motion_start_iter = _state_iter+1;
  // }

  DVec<scalar_t> inter_jpos;
  Eigen::Map<DVec<scalar_t>> kp_joint(
    _data->params->joint_kp.data(), _data->params->joint_kp.size()),
    kd_joint(_data->params->joint_kd.data(), _data->params->joint_kd.size());

  inter_jpos = _SetJPosInterPts(curr_iter, rollover_ramp_iter, initial_jpos, rolling_jpos);
  _data->low_cmd->qd = inter_jpos;
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(inter_jpos - _data->low_state->q) +
                            kd_joint.cwiseProduct(-_data->low_state->dq);
  _data->low_cmd->tau_cmd(3) = _data->low_cmd->tau_cmd(7) = 0.0;
  for (Eigen::Index i(0); i < initial_jpos.size(); ++i) {
    bound(_data->low_cmd->tau_cmd(i), _data->params->torque_limit[i]);
  }
}

void FSMState_RecoveryStand::_StandUp(const int & curr_iter)
{
  Eigen::Map<DVec<scalar_t>> kp_joint(
    _data->params->joint_kp.data(), _data->params->joint_kp.size()),
    kd_joint(_data->params->joint_kd.data(), _data->params->joint_kd.size());

  DVec<scalar_t> inter_jpos =
    _SetJPosInterPts(curr_iter, standup_ramp_iter, initial_jpos, stand_jpos);
  _data->low_cmd->qd = inter_jpos;
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(inter_jpos - _data->low_state->q) +
                            kd_joint.cwiseProduct(-_data->low_state->dq);
  _data->low_cmd->tau_cmd(3) = _data->low_cmd->tau_cmd(7) = 0;

  for (Eigen::Index i(0); i < initial_jpos.size(); ++i) {
    bound(_data->low_cmd->tau_cmd(i), _data->params->torque_limit[i]);
  }
}

void FSMState_RecoveryStand::_FoldLegs(const int & curr_iter)
{
  Eigen::Map<DVec<scalar_t>> kp_joint(
    _data->params->joint_kp.data(), _data->params->joint_kp.size()),
    kd_joint(_data->params->joint_kd.data(), _data->params->joint_kd.size());

  DVec<scalar_t> inter_jpos = _SetJPosInterPts(curr_iter, fold_ramp_iter, initial_jpos, fold_jpos);
  _data->low_cmd->qd = inter_jpos;
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd = kp_joint.cwiseProduct(inter_jpos - _data->low_state->q) +
                            kd_joint.cwiseProduct(-_data->low_state->dq);
  _data->low_cmd->tau_cmd(3) = _data->low_cmd->tau_cmd(7) = 0;

  for (Eigen::Index i(0); i < initial_jpos.size(); ++i) {
    bound(_data->low_cmd->tau_cmd(i), _data->params->torque_limit[i]);
  }

  if (curr_iter >= fold_ramp_iter) {
    if (_UpsideDown()) {
      _flag = RollOver;
      initial_jpos = fold_jpos;
    } else {
      _flag = StandUp;
      initial_jpos = fold_jpos;
    }
    _motion_start_iter = _state_iter + 1;
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */

FSMStateName FSMState_RecoveryStand::checkTransition()
{
  this->_nextStateName = this->_stateName;
  iter++;

  // Switch FSM control mode
  switch (_data->state_command->desire_data_->fsm_state_name) {
    case FSMStateName::RECOVERY_STAND:
      break;

    case FSMStateName::PASSIVE:  // normal c
      this->_nextStateName = FSMStateName::PASSIVE;
      break;

    case FSMStateName::BALANCE_STAND:
      if ((int)(_state_iter - fold_ramp_iter - standup_ramp_iter) >= 100)
        this->_nextStateName = FSMStateName::BALANCE_STAND;
      break;

    case FSMStateName::RL:  // normal c
      if ((int)(_state_iter - fold_ramp_iter - standup_ramp_iter) >= 100)
        this->_nextStateName = FSMStateName::RL;
      break;
    default:
      break;
      // default:
      // std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
      //           << K_RECOVERY_STAND << " to "
      //           << this->_data->controlParameters->control_mode << std::endl;
  }
  // Get the next state
  return this->_nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */

// TransitionData FSMState_RecoveryStand::transition() {
//   // Finish Transition
//   switch (this->_nextStateName) {
//     case FSMStateName::PASSIVE:  // normal
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::BALANCE_STAND:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::LOCOMOTION:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::BACKFLIP:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::FRONTJUMP:
//       this->transitionData.done = true;
//       break;

//     case FSMStateName::VISION:
//       this->transitionData.done = true;
//       break;

//     default:
//       std::cout << "[CONTROL FSM] Something went wrong in transition"
//                 << std::endl;
//   }

//   // Return the transition data to the FSM
//   return this->transitionData;
// }

/**
 * Cleans up the state information on exiting the state.
 */

void FSMState_RecoveryStand::exit()
{
  // Nothing to clean up when exiting
}
