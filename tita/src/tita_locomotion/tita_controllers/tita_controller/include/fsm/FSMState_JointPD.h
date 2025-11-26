#ifndef FSM_TEST_H
#define FSM_TEST_H

#include "FSMState.h"

class FSMState_JointPD : public FSMState
{
public:
  FSMState_JointPD(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_JointPD() {}
  void enter();
  void run();
  void exit();
  FSMStateName checkTransition();

private:
  // data
  DVec<scalar_t> initial_jpos;
};

#endif