#include "planner/DesiredStateCommand.h"
/*=========================== Gait Data ===============================*/




void DesiredStateCommand::setup_state_command()
{
  dt = (scalar_t)1.0f / parameters->update_rate;

  RemoterCommandPlanner::Limits twist_linear_limits[3], twist_angular_limits[3],
    pose_position_limits[3], pose_orientation_limits[3], two_wheel_distance_limits;
  // clang-format off
  twist_linear_limits[point::X] = RemoterCommandPlanner::Limits{
    nullptr,
    nullptr,
    &parameters->velocity_x_max,
    &parameters->velocity_x_min,
    &parameters->acceleration_x_max,
    &parameters->acceleration_x_min};

  twist_angular_limits[point::Z] =
    RemoterCommandPlanner::Limits{
      nullptr,
      nullptr,
      &parameters->velocity_yaw_max,
      nullptr,//&parameters->velocity_yaw_min,
      &parameters->acceleration_yaw_max,
      nullptr};//&parameters->acceleration_yaw_min},

  pose_position_limits[point::Y] =
    RemoterCommandPlanner::Limits{
      &parameters->position_ybias_max,     nullptr,
      &parameters->velocity_ybias_max,     nullptr,
      &parameters->acceleration_ybias_max, nullptr},

  pose_position_limits[point::Z] =
    RemoterCommandPlanner::Limits{
      &parameters->position_height_max,     &parameters->position_height_min,
      &parameters->velocity_height_max,     nullptr,//&parameters->velocity_height_min,
      &parameters->acceleration_height_max, nullptr};//&parameters->acceleration_height_min},

  pose_orientation_limits[rpy::YAW] =
    RemoterCommandPlanner::Limits{
      &parameters->position_xsplit_max,     nullptr,
      &parameters->velocity_xsplit_max,     nullptr,
      &parameters->acceleration_xsplit_max, nullptr},

  pose_orientation_limits[rpy::ROLL] =
    RemoterCommandPlanner::Limits{
      &parameters->position_roll_max,     nullptr,
      &parameters->velocity_roll_max,     nullptr,
      &parameters->acceleration_roll_max, nullptr},

  pose_orientation_limits[rpy::PITCH] = RemoterCommandPlanner::Limits{
    &parameters->position_pitch_max,     nullptr,
    &parameters->velocity_pitch_max,     nullptr,
    &parameters->acceleration_pitch_max, nullptr};

  two_wheel_distance_limits = RemoterCommandPlanner::Limits{
    &parameters->position_ysplit_max,     &parameters->position_ysplit_min,
    &parameters->velocity_ysplit_max,     &parameters->velocity_ysplit_min,
    &parameters->acceleration_ysplit_max, &parameters->acceleration_ysplit_min};

  for (size_t id = 0; id < 3; id++) {
    twist_linear_plan_[id] = RemoterCommandPlanner(
      {RemoterCommandPlanner::RemoteCommand::VELOCITY}, twist_linear_limits[id]);
    twist_angular_plan_[id] = RemoterCommandPlanner(
      {RemoterCommandPlanner::RemoteCommand::VELOCITY}, twist_angular_limits[id]);
    pose_position_plan_[id] = RemoterCommandPlanner(
      {RemoterCommandPlanner::RemoteCommand::POSITION}, pose_position_limits[id]);
    pose_rpy_plan_[id] = RemoterCommandPlanner(
      {RemoterCommandPlanner::RemoteCommand::POSITION}, pose_orientation_limits[id]);
  }
  pose_position_plan_[point::Z] = RemoterCommandPlanner(
    {RemoterCommandPlanner::RemoteCommand::POSITION}, pose_position_limits[point::Z]);
  two_wheel_distance_plan_ = RemoterCommandPlanner(
    {RemoterCommandPlanner::RemoteCommand::POSITION}, two_wheel_distance_limits);  // clang-format on
}

void DesiredStateCommand::convertFSMState()
{
  if (rc_data_->fsm_name_ == "idle") {
    desire_data_->fsm_state_name = FSMStateName::PASSIVE;
  } else if (rc_data_->fsm_name_ == "transform_up") {
    desire_data_->fsm_state_name = FSMStateName::RECOVERY_STAND;
  } else if (rc_data_->fsm_name_ == "balance_stand") {
    desire_data_->fsm_state_name = FSMStateName::BALANCE_STAND;
  } else if (rc_data_->fsm_name_ == "transform_down") {
    desire_data_->fsm_state_name = FSMStateName::TRANSFORM_DOWN;
    // } else if (rc_data_->fsm_name_ == "emergency_stop") {
  } else if (rc_data_->fsm_name_ == "joint_pd") {
    desire_data_->fsm_state_name = FSMStateName::JOINT_PD;
  } else if (rc_data_->fsm_name_ == "rl") {
    desire_data_->fsm_state_name = FSMStateName::RL;
  }
} 
void DesiredStateCommand::convertToStateCommands(std::shared_ptr<RemoteControlData> data)
{
  if (data == nullptr) {
    rcToCommands(rc_data_);
  } else {
    rcToCommands(data);
  }
}
void DesiredStateCommand::clear()
{
  desire_data_->zero();
  // desire_data_->twist_linear_int(point::X) = legWheelsData_->twist_linear_int(point::X);
  // desire_data_->twist_angular_int(point::Z) = stateEstimate->rpy(rpy::YAW);
  // desire_data_->two_wheel_distance = parameters->two_wheel_distance;
  // desire_data_->pose_position(point::Z) = parameters->position_height_min;
  for (size_t id = 0; id < 3; id++) {
    twist_linear_plan_[id].clear();
    twist_angular_plan_[id].clear();
    pose_position_plan_[id].clear();
    pose_rpy_plan_[id].clear();
  }
  two_wheel_distance_plan_.clear();
}

void DesiredStateCommand::rcToCommands(std::shared_ptr<RemoteControlData> rc_data)
{
  // clear x and yaw
  if (firstRun) {
    firstRun = false;
    desire_data_->twist_linear_int(point::X) = legWheelsData_->twist_linear_int(point::X);
    desire_data_->twist_angular_int(point::Z) = stateEstimate->rpy(rpy::YAW);
  }

  // base twist
  twist_linear_plan_[point::X].update(rc_data->twist_linear[point::X], dt);
  twist_angular_plan_[point::Z].update(rc_data->twist_angular[point::Z], dt);
  // base orientation
  vector3_t rpy = ori::quatToRPY(rc_data->pose_orientation);
  pose_rpy_plan_[rpy::ROLL].update(rpy[rpy::ROLL], dt);
  pose_rpy_plan_[rpy::PITCH].update(rpy[rpy::PITCH], dt);
  pose_rpy_plan_[rpy::YAW].update(rpy[rpy::YAW], dt);
  // base position
  pose_position_plan_[point::Y].update(rc_data->pose_position[point::Y], dt);
  pose_position_plan_[point::Z].update(rc_data->pose_position[point::Z], dt);
  // two wheel distance
  two_wheel_distance_plan_.update(rc_data->two_wheel_distance, dt);  // TODO: get from config
  for (size_t id = 0; id < 3; id++) {
    // desire_data_->twist_linear_int(id) = twist_linear_plan_[id].getControlCommand().position;
    // desire_data_->twist_angular_int(id) = twist_angular_plan_[id].getControlCommand().position;
    desire_data_->twist_linear(id) = twist_linear_plan_[id].getControlCommand().velocity;
    desire_data_->twist_angular(id) = twist_angular_plan_[id].getControlCommand().velocity;

    desire_data_->pose_position(id) = pose_position_plan_[id].getControlCommand().position;
    desire_data_->pose_rpy(id) = pose_rpy_plan_[id].getControlCommand().position;
    desire_data_->pose_position_dot(id) = pose_position_plan_[id].getControlCommand().velocity;
    desire_data_->pose_rpy_dot(id) = pose_rpy_plan_[id].getControlCommand().velocity;
  }
  desire_data_->two_wheel_distance =
    two_wheel_distance_plan_.getControlCommand().position + parameters->two_wheel_distance;
  desire_data_->two_wheel_distance_dot = two_wheel_distance_plan_.getControlCommand().velocity;

  desire_data_->twist_linear_int(point::X) =
    desire_data_->twist_linear_int(point::X) /*legWheelsData_->twist_linear_int(point::X)*/ +
    desire_data_->twist_linear(point::X) * dt;
  desire_data_->twist_angular_int(point::Z) =
    desire_data_->twist_angular_int(point::Z) + desire_data_->twist_angular(point::Z) * dt;

  // add bias
  // if (rc_data_->twist_angular[point::Z] > 0) {
  //   data.stateDes(X_SPLIT) += parameters->bias_dif_x_;
  // } else if (rc_data_->twist_angular[point::Z] < 0) {
  //   data.stateDes(X_SPLIT) -= parameters->bias_dif_x_;
  // }
}
