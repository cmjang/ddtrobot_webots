#ifndef DESIRED_STATE_COMMAND_H
#define DESIRED_STATE_COMMAND_H

#include "common/cppTypes.h"
#include "estimator/StateEstimatorContainer.h"
#include "planner/RemoterCommandPlanner.h"

class DesiredStateCommand
{
public:
  struct RemoteControlData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RemoteControlData() { zero(); }
    std::string fsm_name_;
    vector3_t twist_linear;
    vector3_t twist_angular;
    vector3_t pose_position;
    quaternion_t pose_orientation;
    scalar_t two_wheel_distance;
    void zero()
    {
      fsm_name_ = "idle";
      twist_linear.Zero();
      twist_angular.Zero();
      pose_position.Zero();
      pose_orientation.Zero();
      pose_orientation << 1, 0, 0, 0;
      two_wheel_distance = 0;
    }
  };
  struct DesiredStateData
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DesiredStateData() { zero(); }
    FSMStateName fsm_state_name;
    vector3_t twist_linear, twist_linear_int;
    vector3_t twist_angular, twist_angular_int;
    vector3_t pose_position, pose_position_dot;
    vector3_t pose_rpy, pose_rpy_dot;
    scalar_t two_wheel_distance, two_wheel_distance_dot;
    void zero()
    {
      twist_linear.setZero();
      twist_linear_int.setZero();
      twist_angular.setZero();
      twist_angular_int.setZero();
      pose_position.setZero();
      pose_position_dot.setZero();
      pose_rpy.setZero();
      pose_rpy_dot.setZero();
      two_wheel_distance = 0;
      two_wheel_distance_dot = 0;
    };
  };

  DesiredStateCommand(
    std::shared_ptr<RobotControlParameters> _parameters, std::shared_ptr<StateEstimate> sEstimate,
    std::shared_ptr<WheelLeggedData> legWheelsData)
  {
    rc_data_ = std::make_shared<RemoteControlData>();
    desire_data_ = std::make_shared<DesiredStateData>();
    stateEstimate = sEstimate;
    parameters = _parameters;
    legWheelsData_ = legWheelsData;

    twist_linear_plan_.resize(3);
    twist_angular_plan_.resize(3);
    pose_position_plan_.resize(3);
    pose_rpy_plan_.resize(3);
  }
  void setup_state_command();
  void convertFSMState();
  void convertToStateCommands(std::shared_ptr<RemoteControlData> data = nullptr);
  void clear();

  std::shared_ptr<RemoteControlData> rc_data_;
  std::shared_ptr<DesiredStateData> desire_data_;

  std::vector<RemoterCommandPlanner> twist_linear_plan_, twist_angular_plan_;
  std::vector<RemoterCommandPlanner> pose_position_plan_, pose_rpy_plan_;
  RemoterCommandPlanner two_wheel_distance_plan_;

  bool firstRun = true;

private:
  void rcToCommands(std::shared_ptr<RemoteControlData> rc_data);
  std::shared_ptr<StateEstimate> stateEstimate;
  std::shared_ptr<RobotControlParameters> parameters;
  std::shared_ptr<WheelLeggedData> legWheelsData_;
  scalar_t dt;
};

#endif
