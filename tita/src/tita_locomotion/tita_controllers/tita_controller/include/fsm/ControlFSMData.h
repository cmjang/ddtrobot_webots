#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "common/RobotParameters.h"
#include "common/enumClass.h"
#include "estimator/StateEstimatorContainer.h"
#include "planner/DesiredStateCommand.h"
struct ControlFSMData
{
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ControlFSMData(size_t size)
  {
    size_t contacts_num = 2;
    state_estimate = std::make_shared<StateEstimate>();
    wheel_legged_data = std::make_shared<WheelLeggedData>(contacts_num);

    params = std::make_shared<RobotControlParameters>();
    low_state = std::make_shared<LowlevelState>(size);
    low_cmd = std::make_shared<LowlevelCmd>(size);

    state_command =
      std::make_shared<DesiredStateCommand>(params, state_estimate, wheel_legged_data);
    state_estimator = std::make_shared<StateEstimatorContainer>(
      params, low_state, wheel_legged_data, state_estimate);
    // pinocchio_model = std::make_shared<PinocchioInterface>(
    //   params, low_state, low_cmd, state_estimate, state_command, wheel_legged_data);
    // lqr = std::make_shared<LqrBase>(params, wheel_legged_data);
  };

  std::shared_ptr<RobotControlParameters> params;
  // std::shared_ptr<PinocchioInterface> pinocchio_model;
  // std::shared_ptr<LqrBase> lqr;
  std::shared_ptr<LowlevelCmd> low_cmd;
  std::shared_ptr<LowlevelState> low_state;
  std::shared_ptr<StateEstimatorContainer> state_estimator;
  std::shared_ptr<DesiredStateCommand> state_command;

  std::shared_ptr<StateEstimate> state_estimate;
  std::shared_ptr<WheelLeggedData> wheel_legged_data;
};

#endif  // CONTROLFSM_H