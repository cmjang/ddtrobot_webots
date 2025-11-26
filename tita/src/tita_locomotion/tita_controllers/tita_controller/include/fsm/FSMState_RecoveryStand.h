#ifndef FSMSTATE_RECOVERY_STANDUP_H
#define FSMSTATE_RECOVERY_STANDUP_H

#include "FSMState.h"

/**
 *
 */

class FSMState_RecoveryStand : public FSMState
{
public:
  FSMState_RecoveryStand(std::shared_ptr<ControlFSMData> data);
  virtual ~FSMState_RecoveryStand() {}

  // Behavior to be carried out when entering a state
  void enter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSMStateName checkTransition();

  // Manages state specific transitions
  //   TransitionData transition();

  // Behavior to be carried out when exiting a state
  void exit();

  //   TransitionData testTransition();

private:
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  static constexpr int StandUp = 0;
  static constexpr int FoldLegs = 1;
  static constexpr int RollOver = 2;

  unsigned long long _state_iter;
  int _flag = FoldLegs;

  // JPos
  DVec<scalar_t> fold_jpos;
  DVec<scalar_t> stand_jpos;
  DVec<scalar_t> rolling_jpos;
  DVec<scalar_t> initial_jpos;

  DVec<scalar_t> f_ff;

  const float timer_fold = 2.0;  // timer unit : second
  const float timer_standup = 1.0;
  const float timer_rollover = 1.0;
  int fold_ramp_iter, rollover_ramp_iter, standup_ramp_iter;

  void _RollOver(const int & iter);
  void _StandUp(const int & iter);
  void _FoldLegs(const int & iter);

  bool _UpsideDown();
  DVec<scalar_t> _SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, const DVec<scalar_t> & ini,
    const DVec<scalar_t> & fin);
};

#endif  // FSMSTATE_RECOVERY_STANDUP_H
