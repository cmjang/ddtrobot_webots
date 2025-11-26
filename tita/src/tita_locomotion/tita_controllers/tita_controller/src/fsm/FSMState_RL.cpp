/*============================= RL ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "fsm/FSMState_RL.h"
#include "common/timeMarker.h"
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

FSMState_RL::FSMState_RL(std::shared_ptr<ControlFSMData> data)
: FSMState(data, FSMStateName::RL, "rl"),input_0(new float[33]),output(new float[8]),output_last(new float[8])
{
  policy_stand = std::make_shared<CudaTest>("/mnt/dev/stand.engine");
  std::cout << "policy_stand init :" << policy_stand->get_cuda_init() << std::endl;
}

void FSMState_RL::enter()
{ 
    _data->state_command->firstRun = true;
    obs_.dof_pos[3] = _data->low_state->q[0];
    obs_.dof_pos[4] = _data->low_state->q[1];
    obs_.dof_pos[5] = _data->low_state->q[2];

    obs_.dof_pos[0] = _data->low_state->q[4];
    obs_.dof_pos[1] = _data->low_state->q[5];
    obs_.dof_pos[2] = _data->low_state->q[6];

    obs_.dof_pos[7] = _data->low_state->q[3];
    obs_.dof_pos[6] = _data->low_state->q[7];
    // 速度
    obs_.dof_vel[3] = _data->low_state->dq[0];
    obs_.dof_vel[4] = _data->low_state->dq[1];
    obs_.dof_vel[5] = _data->low_state->dq[2];

    obs_.dof_vel[0] = _data->low_state->dq[4];
    obs_.dof_vel[1] = _data->low_state->dq[5];
    obs_.dof_vel[2] = _data->low_state->dq[6];

    obs_.dof_vel[7] = _data->low_state->dq[3];
    obs_.dof_vel[6] = _data->low_state->dq[7];

  // RL 的轮子：6、7；观测里轮子位置按 0
  obs_.dof_pos[6] = 0.0f;
  obs_.dof_pos[7] = 0.0f;
  params_.action_scale = 0.25;
  params_.num_of_dofs = 8;
  params_.lin_vel_scale = 2.0;
  params_.ang_vel_scale = 0.25;
  params_.dof_pos_scale = 1.0;
  params_.dof_vel_scale = 0.05;

  params_.commands_scale[0] = 1.0;
  params_.commands_scale[1] = 1.0;
  params_.commands_scale[2] = 1.0;

  const float default_dof_pos_tmp[8] = {0, 0.8, -1.5,
                                        0, 0.8,-1.5,
                                        0, 0};
  for (int i = 0; i < 8; i++)
  {
    params_.default_dof_pos[i] = default_dof_pos_tmp[i];
  }

  x_vel_cmd_ = 0.;
  y_vel_cmd_ = 0.;
  pitch_cmd_ = 0.;

  for (int i = 0; i < 8; i++)
    output_last.get()[i] = 0;

  obs_.forward_vec[0] = 1.0;
  obs_.forward_vec[1] = 0.0;
  obs_.forward_vec[2] = 0.0;

  for (int j = 0; j < 8; j++)
  {
    action[j] = obs_.dof_pos[j];
  }
  a_l.setZero();

  std::cout << "init finised predict" << std::endl;

  for (int i = 0; i < 10; i++)
  {
    _Forward();
  }

  threadRunning = true;
  if (thread_first_)
  {
    forward_thread = std::thread(&FSMState_RL::_Run_Forward, this);
    thread_first_ = false;
  }
  stop_update_ = false;
}

void FSMState_RL::run()
{
  // _data->state_command->clear();
  // _data->low_cmd->zero();
  x_vel_cmd_ = _data->state_command->rc_data_->twist_linear[point::X];
  y_vel_cmd_ = _data->state_command->rc_data_->twist_linear[point::Y];
  pitch_cmd_ = _data->state_command->rc_data_->twist_angular[point::Z];
  // _data->state_command->rc_data_->twist_angular[point::Z]
  _data->low_cmd->qd.setZero();
  _data->low_cmd->qd_dot.setZero();
  _data->low_cmd->kp.setZero();
  _data->low_cmd->kd.setZero();
  _data->low_cmd->tau_cmd.setZero();

  _data->low_cmd->tau_cmd[0] = 40 * (desired_pos[0]- _data->low_state->q[0]) +  1.0f * (0 - _data->low_state->dq[0]);  //左腿
  _data->low_cmd->tau_cmd[1] = 40 * (desired_pos[1]- _data->low_state->q[1]) +  1.0f * (0 - _data->low_state->dq[1]);
  _data->low_cmd->tau_cmd[2] = 40 * (desired_pos[2]- _data->low_state->q[2]) +  1.0f * (0 - _data->low_state->dq[2]);

  _data->low_cmd->tau_cmd[4] = 40 * (desired_pos[3]- _data->low_state->q[4]) +  1.0f * (0 - _data->low_state->dq[4]);  //右腿
  _data->low_cmd->tau_cmd[5] = 40 * (desired_pos[4]- _data->low_state->q[5]) +  1.0f * (0 - _data->low_state->dq[5]);
  _data->low_cmd->tau_cmd[6] = 40 * (desired_pos[5]- _data->low_state->q[6]) +  1.0f * (0 - _data->low_state->dq[6]);

  _data->low_cmd->tau_cmd[3] = 2.0f  * (desired_pos[6] - _data->low_state->dq[3]);  //left 
  _data->low_cmd->tau_cmd[7] = 2.0f  * (desired_pos[7] - _data->low_state->dq[7]);  
}
void FSMState_RL::exit() 
{
  stop_update_ = true;
}

FSMStateName FSMState_RL::checkTransition()
{
  this->_nextStateName = this->_stateName;

  // Switch FSM control mode
  switch (_data->state_command->desire_data_->fsm_state_name)
  {
  case FSMStateName::RECOVERY_STAND:
    this->_nextStateName = FSMStateName::RECOVERY_STAND;
    break;

  case FSMStateName::RL: // normal c
    break;

  case FSMStateName::TRANSFORM_DOWN:
    this->_nextStateName = FSMStateName::TRANSFORM_DOWN;
    break;

  case FSMStateName::PASSIVE: // normal c
    this->_nextStateName = FSMStateName::PASSIVE;
    break;
  default:
    break;
  }
  return this->_nextStateName;
}

void FSMState_RL::_GetObs()
{
    std::vector<float> obs_tmp;
    // compute gravity
    Mat3<double> _B2G_RotMat = this->_data->state_estimator->getResult().rBody;
    Mat3<double> _G2B_RotMat = this->_data->state_estimator->getResult().rBody.transpose();

    Vec3<double> angvel = a_l;
    a_l = 0.97*this->_data->state_estimator->getResult().omegaBody + 0.03*a_l;
    Vec3<double> projected_gravity = _B2G_RotMat * Vec3<double>(0.0, 0.0, -1.0);
    // gravity
    obs_tmp.push_back(_data->low_state->gyro(0)*params_.ang_vel_scale);
    obs_tmp.push_back(_data->low_state->gyro(1)*params_.ang_vel_scale);
    obs_tmp.push_back(_data->low_state->gyro(2)*params_.ang_vel_scale);

    for (int i = 0; i < 3; ++i)
    {
        obs_tmp.push_back(projected_gravity(i));
    }
   // std::cout<<"projected_gravity: "<<projected_gravity(0)<<" "<<projected_gravity(1)<<" "<<projected_gravity(2)<<std::endl;
    // cmd
    float rx = pitch_cmd_;
    float lx = x_vel_cmd_;
    float ly = y_vel_cmd_;

    float max = 1.0;
    float min = -1.0;

    float rot = rx*3.14;
    float vel_x = lx*2;
    float vel_y = ly*2;

    double heading = 0.;
    double angle = (double)rot - heading;
    angle = fmod(angle,2.0*M_PI);
    if(angle > M_PI)
    {
        angle = angle - 2.0*M_PI;
    }
    angle = angle*0.5;
    angle = std::max(std::min((float)angle, max), min);
    angle = angle * 0.25;

    obs_tmp.push_back(vel_x);
    obs_tmp.push_back(vel_y);
    obs_tmp.push_back(angle);
//=================================================

    this->obs_.dof_pos[6]=0;
    this->obs_.dof_pos[7]=0;
    // pos
    for (int i = 0; i < 8; ++i)
    {
        float pos = (this->obs_.dof_pos[i]  - this->params_.default_dof_pos[i]) * params_.dof_pos_scale;
        obs_tmp.push_back(pos);
    }
    // vel
    for (int i = 0; i < 8; ++i)
    {
        float vel = this->obs_.dof_vel[i] * params_.dof_vel_scale;
        obs_tmp.push_back(vel);
    }

    // last action
    //float index[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
    for (int i = 0; i < 8; ++i)
    {
        obs_tmp.push_back(output_last.get()[i]);
    }
    for(int i = 0; i < 33; i++)
    {
        input_0.get()[i] = obs_tmp[i];
    }

}

void FSMState_RL::_Forward()
{
    _GetObs();
    policy_stand->do_inference(input_0.get(), output.get());   //stand
    for (int i = 0; i < 8; i++)
        output_last.get()[i] = output.get()[i];

}

inline long long now_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000;
}


int discrime =0;
void FSMState_RL::_Run_Forward()
{
  while (threadRunning)
  {
    long long _start_time = getSystemTime();
    // auto t0 = std::chrono::high_resolution_clock::now();
    if (!stop_update_)
    {
      obs_.dof_pos[3] = _data->low_state->q[0];
      obs_.dof_pos[4] = _data->low_state->q[1];
      obs_.dof_pos[5] = _data->low_state->q[2];

      obs_.dof_pos[0] = _data->low_state->q[4];
      obs_.dof_pos[1] = _data->low_state->q[5];
      obs_.dof_pos[2] = _data->low_state->q[6];

      obs_.dof_pos[6] = 0.0f;
      obs_.dof_pos[7] = 0.0f;
      // 速度
      obs_.dof_vel[3] = _data->low_state->dq[0];
      obs_.dof_vel[4] = _data->low_state->dq[1];
      obs_.dof_vel[5] = _data->low_state->dq[2];

      obs_.dof_vel[0] = _data->low_state->dq[4];
      obs_.dof_vel[1] = _data->low_state->dq[5];
      obs_.dof_vel[2] = _data->low_state->dq[6];

      obs_.dof_vel[7] = _data->low_state->dq[3];
      obs_.dof_vel[6] = _data->low_state->dq[7];

      _Forward();

      action[0] =  params_.default_dof_pos[0]  + output.get()[0] * 0.125f;  //左腿
      action[1] =  params_.default_dof_pos[1] + output.get()[1] * 0.25f;   
      action[2] =  params_.default_dof_pos[2] + output.get()[2] * 0.25f;
      
      action[3] =  params_.default_dof_pos[3] + output.get()[3] * 0.125f;
      action[4] =  params_.default_dof_pos[4] + output.get()[4] * 0.25f;
      action[5] =  params_.default_dof_pos[5] + output.get()[5] * 0.25f;
      action[6] =  output.get()[6] * 5.0f;
      action[7] =  output.get()[7] * 5.0f;

      for (int i = 0; i < 3; i++)
      {
        desired_pos[i+3] = action[i];
        desired_pos[i] = action[i+3];
      }
      desired_pos[6] = action[7];
      desired_pos[7] = action[6];

      episode_time+=dt;   //time

    }
    absoluteWait(_start_time, (long long)(0.02 * 10000*100));
  }
  threadRunning = false;
}