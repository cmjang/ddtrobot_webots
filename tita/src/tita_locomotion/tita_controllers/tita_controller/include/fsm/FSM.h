#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_JointPD.h"
#include "FSMState_Passive.h"
#include "FSMState_RecoveryStand.h"
#include "FSMState_TransformDown.h"
#include "FSMState_RL.h"
#include "common/enumClass.h"

struct FSMStateList
{
  FSMState * invalid;
  FSMState_Passive * passive;
  // FSMState_BalanceStand * balanceStand;
  FSMState_RecoveryStand * recoveryStand;
  FSMState_JointPD * jointPD;
  FSMState_TransformDown * transformDown;
  FSMState_RL * rl;

  void deletePtr()
  {
    delete invalid;
    delete passive;
    // delete balanceStand;
    delete recoveryStand;
    delete jointPD;
    delete transformDown;
    delete rl;
  }
};

class FSM
{
public:
  FSM(std::shared_ptr<ControlFSMData> data);
  ~FSM();
  void initialize();
  void run();
  std::string getCurrentStateName() { return _currentState->_stateNameStr; };

private:
  FSMState * getNextState(FSMStateName stateName);
  bool checkSafty();
  std::shared_ptr<ControlFSMData> _data;
  FSMState * _currentState;
  FSMState * _nextState;
  FSMStateName _nextStateName;
  FSMStateList _stateList;
  FSMMode _mode;
  long long _startTime;
  int count;
};

#endif