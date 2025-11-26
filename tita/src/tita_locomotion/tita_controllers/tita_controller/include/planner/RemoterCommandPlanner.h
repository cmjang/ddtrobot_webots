#ifndef REMOTER_COMMAND_PLANNER_H
#define REMOTER_COMMAND_PLANNER_H

#include <math.h>
#include <stdio.h>

class RemoterCommandPlanner
{
public:
  struct ControlCommand
  {
    scalar_t position;
    scalar_t velocity;
    scalar_t acceleration;
    ControlCommand() { zero(); }
    void zero() { position = velocity = acceleration = 0.0; }
  };

  struct RemoteCommand
  {
    enum Type { POSITION, VELOCITY } type;
    scalar_t value;
    RemoteCommand(Type type, scalar_t value) : type(type), value(value) {}
    RemoteCommand(Type type) : type(type), value(0.0) {}
    RemoteCommand() : type(Type::POSITION), value(0.0) {}
    void zero() { value = 0.0; }
  };

  struct Limits
  {
    scalar_t * maxPosition{nullptr};
    scalar_t * minPosition{nullptr};
    scalar_t * maxVelocity{nullptr};
    scalar_t * minVelocity{nullptr};
    scalar_t * maxAcceleration{nullptr};
    scalar_t * minAcceleration{nullptr};
  };
  // 限制值在最小值和最大值之间
  scalar_t clamp(scalar_t value, double min, double max)
  {
    return std::max(min, std::min(value, max));
  }

  RemoterCommandPlanner(RemoteCommand remote_cmd, Limits limits)
  {
    remote_cmd_ = remote_cmd;
    limits_ = limits;
  }
  RemoterCommandPlanner() {}
  ~RemoterCommandPlanner() {}
  void clear()
  {
    control_cmd_.zero();
    remote_cmd_.zero();
  }
  void update(scalar_t input, scalar_t dt)
  {
    remote_cmd_.value = input;
    scalar_t desiredPosition, desiredVelocity, desiredAcceleration;
    scalar_t minPosition, maxPosition, minVelocity, maxVelocity, minAcceleration, maxAcceleration;
    maxPosition = limits_.maxPosition == nullptr ? 0.0 : *limits_.maxPosition;
    maxVelocity = limits_.maxVelocity == nullptr ? 0.0 : *limits_.maxVelocity;
    maxAcceleration = limits_.maxAcceleration == nullptr ? 0.0 : *limits_.maxAcceleration;

    minPosition = limits_.minPosition == nullptr ? -maxPosition : *limits_.minPosition;
    minVelocity = limits_.minVelocity == nullptr ? -maxVelocity : *limits_.minVelocity;
    minAcceleration =
      limits_.minAcceleration == nullptr ? -maxAcceleration : *limits_.minAcceleration;

    switch (remote_cmd_.type) {
      case RemoteCommand::POSITION:
        control_cmd_.position = clamp(control_cmd_.position, minPosition, maxPosition);
        desiredPosition = clamp(remote_cmd_.value, minPosition, maxPosition);
        desiredVelocity = (desiredPosition - control_cmd_.position) / dt;
        desiredVelocity = clamp(desiredVelocity, minVelocity, maxVelocity);
        desiredAcceleration = (desiredVelocity - control_cmd_.velocity) / dt;
        control_cmd_.acceleration = clamp(desiredAcceleration, minAcceleration, maxAcceleration);
        // 更新速度和位置
        control_cmd_.velocity = control_cmd_.velocity + control_cmd_.acceleration * dt;
        control_cmd_.position = control_cmd_.position + control_cmd_.velocity * dt;
        break;

      case RemoteCommand::VELOCITY:  // velocity commond without position limit
        // 输入为速度指令
        desiredVelocity = clamp(remote_cmd_.value, minVelocity, maxVelocity);
        desiredAcceleration = (desiredVelocity - control_cmd_.velocity) / dt;
        control_cmd_.acceleration = clamp(desiredAcceleration, minAcceleration, maxAcceleration);
        // 更新速度和位置
        control_cmd_.velocity = control_cmd_.velocity + control_cmd_.acceleration * dt;
        control_cmd_.position = control_cmd_.position + control_cmd_.velocity * dt;
        break;
    }
  }
  ControlCommand getControlCommand() { return control_cmd_; }

private:
  RemoteCommand remote_cmd_;
  ControlCommand control_cmd_;
  Limits limits_;
};

#endif
