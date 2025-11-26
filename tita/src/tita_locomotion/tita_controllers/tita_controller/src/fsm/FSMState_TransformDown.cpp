/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_TransformDown.h"

FSMState_TransformDown::FSMState_TransformDown(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::TRANSFORM_DOWN, "transform_down")
{
  size_t dof = _data->params->dof_chassis;
  initial_jpos.setZero(dof);
  fold_jpos.setZero(dof);

  if (dof == 8) {
    fold_jpos << 0.5f, 1.20f, -2.7f, 0.0f, -0.5f, 1.2f, -2.7f, 0.0f;
  } else {
    fold_jpos << 1.2f, -2.6f, 0.0f, 1.2f, -2.6f, 0.0f;
  }
  rc_data_ = std::make_shared<DesiredStateCommand::RemoteControlData>();
}

void FSMState_TransformDown::enter()
{
  this->_nextStateName = this->_stateName;
  fold_ramp_iter = timer_fold * _data->params->update_rate;
  _state_iter = 0;
  _data->state_command->firstRun = true;
}

void FSMState_TransformDown::run()
{
  _data->low_cmd->zero();
  if (!balance_finish_) {
    _data->state_command->convertToStateCommands(
      std::make_shared<DesiredStateCommand::RemoteControlData>());
    // _data->pinocchio_model->updateWBC();
    if (
      std::abs(_data->wheel_legged_data->pose_position_dot[point::Z]) < 1e-3 &&
      std::abs(_data->wheel_legged_data->pose_position[point::Z]) <
        _data->params->position_height_min + 0.02) {
      balance_finish_ = true;
      initial_jpos = this->_data->low_state->q;
      fold_jpos(3) = initial_jpos(3);
      fold_jpos(7) = initial_jpos(7);
      fold_ramp_iter = fold_ramp_iter * (initial_jpos - fold_jpos).cwiseAbs().maxCoeff();  //
    }
  } else {
    _FoldLegs(_state_iter);
    ++_state_iter;
  }
}

DVec<scalar_t> FSMState_TransformDown::_SetJPosInterPts(
  const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini, const DVec<scalar_t> & fin)
{
  if (ini.size() != fin.size()) {
    std::cerr << "[FSMState_TransformDown] ERROR: ini and fin must have the same size" << std::endl;
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

void FSMState_TransformDown::_FoldLegs(const int & curr_iter)
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
    //   if (_UpsideDown()) {
    //     _flag = RollOver;
    //     initial_jpos = fold_jpos;
    //   } else {
    //     _flag = StandUp;
    //     initial_jpos = fold_jpos;
    //   }
    transform_finish_ = true;
  }
}

FSMStateName FSMState_TransformDown::checkTransition()
{
  this->_nextStateName = this->_stateName;
  switch (_data->state_command->desire_data_->fsm_state_name) {
    case FSMStateName::RECOVERY_STAND:
      this->_nextStateName = FSMStateName::RECOVERY_STAND;
      break;
    case FSMStateName::PASSIVE:  // normal c
      this->_nextStateName = FSMStateName::PASSIVE;
      break;

    default:
      break;
  }
  if (transform_finish_) {
    this->_nextStateName = FSMStateName::PASSIVE;
  }
  return this->_nextStateName;
}

void FSMState_TransformDown::exit()
{
  balance_finish_ = false;
  transform_finish_ = false;
  // Nothing to clean up when exiting
}
