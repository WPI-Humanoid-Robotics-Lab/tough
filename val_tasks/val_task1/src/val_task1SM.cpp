#include <val_task1/val_task1.h>

// finite state machine name for tackling task-1
FSM(val_task1)
{
  //define the states
  FSM_STATES
  {
    STATE_INIT,
    STATE_WALK_TO_CONTROL,
    STATE_CORRECT_PITCH,
    STATE_CORRECT_YAW,
    STATE_WALK_TO_FINISH,
    STATE_END
  }

  // give the start state
  FSM_START(STATE_INIT);

  // state machine structure and logic, describe the states tansitions
  FSM_BGN // begin state machine
  {
    FSM_STATE(STATE_INIT)
    {
      // state excecution, call the task
      FSM_CALL_TASK(STATE_INIT)

       // state transitions
       FSM_TRANSITIONS
      {
        // create transitions based on the requirment
        // on event
        FSM_ON_EVENT("/INIT_SUCESSUFL", FSM_NEXT(STATE_WALK_TO_CONTROL))
        FSM_ON_EVENT("/INIT_FAILED", FSM_NEXT(STATE_INIT))

        // or on condition or next state directly
      }
    }
    FSM_STATE(STATE_WALK_TO_CONTROL)
    {
      FSM_CALL_TASK(STATE_WALK_TO_CONTROL)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/REACHED_PANEL", FSM_NEXT(STATE_CORRECT_PITCH))
      }
    }
    FSM_STATE(STATE_CORRECT_PITCH)
    {

      FSM_CALL_TASK(STATE_CORRECT_PITCH)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/PITCH_CORRECTION_SUCESSFUL", FSM_NEXT(STATE_CORRECT_YAW))
      }
    }
    FSM_STATE(STATE_CORRECT_YAW)
    {

      FSM_CALL_TASK(STATE_CORRECT_YAW)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/YAW_CORRECTION_SUCESSFUL", FSM_NEXT(STATE_WALK_TO_FINISH))
      }
    }
    FSM_STATE(STATE_WALK_TO_FINISH)
    {

      FSM_CALL_TASK(STATE_WALK_TO_FINISH)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/YAW_CORRECTION_SUCESSFUL", FSM_NEXT(STATE_END))
      }
    }
    FSM_STATE(STATE_END)
    {

      FSM_CALL_TASK(STATE_END)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/RESTART", FSM_NEXT(STATE_INIT))
      }
    }
  }
  FSM_END // end of state machine
}
