#ifndef PASSIVE_H
#define PASSIVE_H

#include "FSMState.h"

class FSMState_Passive : public FSMState
{
public:
  FSMState_Passive(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_Passive() {}

  void enter();
  void run();
  void exit();
  FSMStateName checkTransition();
  FSMStateName fsm_name_pre_;
};

#endif