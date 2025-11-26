#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <iostream>
#include <string>

#include "common/cppTypes.h"
#include "common/enumClass.h"
#include "fsm/ControlFSMData.h"

class FSMState
{
public:
  FSMState(std::shared_ptr<ControlFSMData> data, FSMStateName stateName, std::string stateNameStr)
  : _stateName(stateName),
    _stateNameStr(stateNameStr),
    _data(data){

    };
  virtual ~FSMState() = 0;

  virtual void enter() = 0;
  virtual void run() = 0;
  virtual void exit() = 0;
  virtual FSMStateName checkTransition() { return FSMStateName::INVALID; }

  FSMStateName _stateName;
  std::string _stateNameStr;

protected:
  std::shared_ptr<ControlFSMData> _data;
  FSMStateName _nextStateName;
};

#endif  // FSMSTATE_H