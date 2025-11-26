#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include "cppTypes.h"

enum class FSMStateName {
  INVALID,
  PASSIVE,
  JOINT_PD,
  IMPEDANCE_CONTROL,
  STAND_UP,
  BALANCE_STAND,
  LOCOMOTION,
  RECOVERY_STAND,
  // VISION,
  BACKFLIP,
  FRONTJUMP,
  TRANSFORM_DOWN,
  RL,
};

enum point { X, Y, Z };

enum rpy { ROLL, PITCH, YAW };

enum quat { QW, QX, QY, QZ };

struct LowlevelCmd
{
  LowlevelCmd(size_t size)
  {
    tau_cmd.setZero(size);
    qd.setZero(size);
    qd_dot.setZero(size);
    kp.setZero(size);
    kd.setZero(size);
  }
  void zero()
  {
    tau_cmd.setZero();
    qd.setZero();
    qd_dot.setZero();
    kp.setZero();
    kd.setZero();
  }

  vector_t tau_cmd;
  vector_t qd;
  vector_t qd_dot;
  vector_t kp;
  vector_t kd;
};

struct LowlevelState
{
  LowlevelState(size_t size) { zero(size); }
  void zero(size_t size)
  {
    accelerometer = Vec3<double>::Zero();
    gyro = Vec3<double>::Zero();
    quat << 1, 0, 0, 0;
    q.setZero(size);
    dq.setZero(size);
    tau_est.setZero(size);
  }
  vector3_t accelerometer;
  vector3_t gyro;
  quaternion_t quat;
  vector_t q, dq, tau_est;
};

struct WheelLeggedData
{
  WheelLeggedData(size_t contact_size)
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
    com_position_relative.setZero();
    com_velocity_relative.setZero();
    contact_position_local.resize(contact_size);
    contact_velocity_local.resize(contact_size);
    support_velocity_local.resize(contact_size);
    contact_position_world.resize(contact_size);
    contact_velocity_world.resize(contact_size);
    support_velocity_world.resize(contact_size);
    contact_forces.resize(contact_size);
  }
  std::vector<vector3_t> contact_position_local, contact_velocity_local, support_velocity_local;
  std::vector<vector3_t> contact_position_world, contact_velocity_world, support_velocity_world;
  std::vector<vector3_t> contact_forces;
  // for lqr
  Eigen::MatrixXd K_;
  // new
  FSMStateName fsm_state_name;
  vector3_t twist_linear, twist_linear_int;
  vector3_t twist_angular, twist_angular_int;
  vector3_t pose_position, pose_position_dot;
  vector3_t pose_rpy, pose_rpy_dot;
  scalar_t two_wheel_distance, two_wheel_distance_dot;
  vector3_t com_position_relative, com_velocity_relative;
};
/*
  world frame 世界坐标系, 也为inertial frame, 固定坐标系
  control frame 控制坐标系,表示从机器人body frame左乘没有yaw旋转的变换,
  body frame 机器人坐标系, 固定在机器人的base_link上
*/
struct StateEstimate
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Vec4<double> contactEstimate;
  Vec3<double> position;
  Vec3<double> vBody;
  Quat<double> orientation;
  Vec3<double> omegaBody;
  RotMat<double> rBody;
  Vec3<double> rpy;

  Vec3<double> omegaWorld;
  Vec3<double> vWorld;
  Vec3<double> aBody, aWorld;

  matrix3_t rCtrl;
  vector3_t omegaCtrl, vCtrl, aCtrl;
};

enum class FSMMode { NORMAL, CHANGE };

#endif  // ENUMCLASS_H