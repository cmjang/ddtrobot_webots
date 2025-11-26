#ifndef FSMSTATE_TRANSFORM_DOWN_H
#define FSMSTATE_TRANSFORM_DOWN_H

#include "FSMState.h"
class FSMState_TransformDown : public FSMState
{
public:
  FSMState_TransformDown(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_TransformDown() {}

  void enter();
  void run();
  void exit();
  FSMStateName checkTransition();

private:
  void _FoldLegs(const int & iter);
  DVec<scalar_t> _SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini,
    const DVec<scalar_t> & fin);

  std::shared_ptr<DesiredStateCommand::RemoteControlData> rc_data_;
  unsigned long long _state_iter;
  // JPos
  DVec<scalar_t> fold_jpos, initial_jpos;
  int fold_ramp_iter;
  const float timer_fold = 2.0;  // timer unit : second
  bool balance_finish_ = false;
  bool transform_finish_ = false;
};

#endif  // FSMSTATE_RECOVERY_STANDUP_H
