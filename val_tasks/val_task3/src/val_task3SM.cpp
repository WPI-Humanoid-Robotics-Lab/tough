#include <val_task3/val_task3.h>

// finite state machine name for tackling task-3
FSM(val_task3)
{
  //define the states
  FSM_STATES
  {
    STATE_INIT,
    STATE_DETECT_STAIRS,
    STATE_CLIMB_STAIRS,
    STATE_DETECT_DOOR_HANDLE,
    STATE_OPEN_DOOR,
    STATE_WALK_THROUGH_DOOR,
    STATE_DETECT_LEAK_TOOL,
    STATE_WALK_TO_LEAK_TOOL,
    STATE_PICK_LEAK_TOOL,
    STATE_WALK_TO_DETECT_LEAK_LOCATION,
    STATE_DETECT_LEAK_LOCATION,
    STATE_DETECT_REPAIR_TOOL,
    STATE_WALK_TO_REPAIR_TOOL,
    STATE_PICK_REPAIR_TOOL,
    STATE_WALK_TO_LEAK_LOCATION,
    STATE_LEAK_REPAIR,
    STATE_DETECT_FINISH,
    STATE_WALK_TO_FINISH,
    STATE_END,
    STATE_ERROR
  }

  // give the start state
  FSM_START(STATE_INIT);

  // state machine structure and logic, describe the states tansitions
  FSM_BGN // begin state machine
  {
    FSM_STATE(STATE_INIT)
    {
      FSM_CALL_TASK(STATE_INIT)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/INIT_RETRY", FSM_NEXT(STATE_INIT))
        FSM_ON_EVENT("/INIT_SUCESSFUL", FSM_NEXT(STATE_DETECT_STAIRS))
        FSM_ON_EVENT("/INIT_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_DETECT_STAIRS)
    {
      FSM_CALL_TASK(STATE_DETECT_STAIRS)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/DETECT_STAIRS_RETRY", FSM_NEXT(STATE_DETECT_STAIRS))
        FSM_ON_EVENT("/DETECTED_STAIRS", FSM_NEXT(STATE_CLIMB_STAIRS))
        FSM_ON_EVENT("/DETECT_STAIRS_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_CLIMB_STAIRS)
    {
      FSM_CALL_TASK(STATE_CLIMB_STAIRS)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/CLIMB_STAIRS_EXECUTING", FSM_NEXT(STATE_CLIMB_STAIRS))
        FSM_ON_EVENT("/CLIMBED_STAIRS", FSM_NEXT(STATE_DETECT_DOOR_HANDLE))
        FSM_ON_EVENT("/CLIMB_STAIRS_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_DETECT_DOOR_HANDLE)
    {
      FSM_CALL_TASK(STATE_DETECT_DOOR_HANDLE)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/DOOR_HANDLE_DETECT_RETRY", FSM_NEXT(STATE_DETECT_DOOR_HANDLE))
        FSM_ON_EVENT("/DOOR_HANDLE_DETECTED", FSM_NEXT(STATE_OPEN_DOOR))
        FSM_ON_EVENT("/DOOR_HANDLE_DETECT_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_OPEN_DOOR)
    {
      FSM_CALL_TASK(STATE_OPEN_DOOR)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/DOOR_OPEN_RETRY", FSM_NEXT(STATE_OPEN_DOOR))
        FSM_ON_EVENT("/DOOR_OPEN_EXECUTING", FSM_NEXT(STATE_OPEN_DOOR))
        FSM_ON_EVENT("/DOOR_OPENED", FSM_NEXT(STATE_WALK_THROUGH_DOOR))
        FSM_ON_EVENT("/DOOR_HANDLE_DETECT_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_WALK_THROUGH_DOOR)
    {
      FSM_CALL_TASK(STATE_WALK_THROUGH_DOOR)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/WALK_THROGH_DOOR_EXECUTING", FSM_NEXT(STATE_WALK_THROUGH_DOOR))
        FSM_ON_EVENT("/PASSED_THROUGH_DOOR", FSM_NEXT(STATE_DETECT_LEAK_TOOL))
        FSM_ON_EVENT("/WALK_THROGH_DOOR_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_DETECT_LEAK_TOOL)
    {
      FSM_CALL_TASK(STATE_DETECT_LEAK_TOOL)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/DETECT_LEAK_TOOL_RETRY", FSM_NEXT(STATE_DETECT_LEAK_TOOL))
        FSM_ON_EVENT("/DETECTED_LEAK_TOOL", FSM_NEXT(STATE_WALK_TO_LEAK_TOOL))
        FSM_ON_EVENT("/DETECT_LEAK_TOOL_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_WALK_TO_LEAK_TOOL)
    {
      FSM_CALL_TASK(STATE_WALK_TO_LEAK_TOOL)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/WALK_TO_LEAK_TOOL_FAILED", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/WALK_TO_LEAK_TOOL_EXECUTING", FSM_NEXT(STATE_WALK_TO_LEAK_TOOL))
        FSM_ON_EVENT("/WALK_TO_LEAK_TOOL_RETRY", FSM_NEXT(STATE_DETECT_LEAK_TOOL))
        FSM_ON_EVENT("/REACHED_LEAK_TOOL", FSM_NEXT(STATE_PICK_LEAK_TOOL))
      }
    }
    FSM_STATE(STATE_PICK_LEAK_TOOL)
    {

      FSM_CALL_TASK(STATE_PICK_LEAK_TOOL)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/PICK_LEAK_TOOL_EXECUTING", FSM_NEXT(STATE_PICK_LEAK_TOOL))
        FSM_ON_EVENT("/PICK_LEAK_TOOL_RETRY", FSM_NEXT(STATE_PICK_LEAK_TOOL))
        FSM_ON_EVENT("/PICK_LEAK_TOOL_FAILED", FSM_NEXT(STATE_DETECT_LEAK_TOOL))
        FSM_ON_EVENT("/PICKED_LEAK_TOOL", FSM_NEXT(STATE_WALK_TO_DETECT_LEAK_LOCATION))
      }
    }
    FSM_STATE(STATE_WALK_TO_DETECT_LEAK_LOCATION)
    {
      FSM_CALL_TASK(STATE_WALK_TO_DETECT_LEAK_LOCATION)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/WALK_TO_LEAK_WALL_FAILED", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/WALK_TO_LEAK_WALL_EXECUTING", FSM_NEXT(STATE_WALK_TO_DETECT_LEAK_LOCATION))
        FSM_ON_EVENT("/REACHED_LEAK_WALL", FSM_NEXT(STATE_DETECT_LEAK_LOCATION))
      }
    }
    FSM_STATE(STATE_DETECT_LEAK_LOCATION)
    {

      FSM_CALL_TASK(STATE_DETECT_LEAK_LOCATION)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/LEAK_DETECT_EXECUTING", FSM_NEXT(STATE_DETECT_LEAK_LOCATION))
        FSM_ON_EVENT("/LEAK_DETECT_FAILED", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/LEAK_DETECTED", FSM_NEXT(STATE_DETECT_REPAIR_TOOL))
      }
    }
    FSM_STATE(STATE_DETECT_REPAIR_TOOL)
    {
      FSM_CALL_TASK(STATE_DETECT_REPAIR_TOOL)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/DETECT_REPAIR_TOOL_RETRY", FSM_NEXT(STATE_DETECT_REPAIR_TOOL))
        FSM_ON_EVENT("/DETECTED_REPAIR_TOOL", FSM_NEXT(STATE_WALK_TO_REPAIR_TOOL))
        FSM_ON_EVENT("/DETECT_REPAIR_TOOL_FAILED", FSM_NEXT(STATE_ERROR))
      }
    }
    FSM_STATE(STATE_WALK_TO_REPAIR_TOOL)
    {
      FSM_CALL_TASK(STATE_WALK_TO_REPAIR_TOOL)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/WALK_TO_REPAIR_TOOL_FAILED", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/WALK_TO_REPAIR_TOOL_EXECUTING", FSM_NEXT(STATE_WALK_TO_REPAIR_TOOL))
        FSM_ON_EVENT("/WALK_TO_REPAIR_TOOL_RETRY", FSM_NEXT(STATE_DETECT_REPAIR_TOOL))
        FSM_ON_EVENT("/REACHED_REPAIR_TOOL", FSM_NEXT(STATE_PICK_REPAIR_TOOL))
      }
    }
    FSM_STATE(STATE_PICK_REPAIR_TOOL)
    {

      FSM_CALL_TASK(STATE_PICK_REPAIR_TOOL)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/PICK_REPAIR_TOOL_EXECUTING", FSM_NEXT(STATE_PICK_REPAIR_TOOL))
        FSM_ON_EVENT("/PICK_REPAIR_TOOL_RETRY", FSM_NEXT(STATE_PICK_REPAIR_TOOL))
        FSM_ON_EVENT("/PICK_REPAIR_TOOL_FAILED", FSM_NEXT(STATE_DETECT_REPAIR_TOOL))
        FSM_ON_EVENT("/PICKED_REPAIR_TOOL", FSM_NEXT(STATE_WALK_TO_LEAK_LOCATION))
      }
    }
    FSM_STATE(STATE_WALK_TO_LEAK_LOCATION)
    {
      FSM_CALL_TASK(STATE_WALK_TO_LEAK_LOCATION)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/WALK_TO_LEAK_LOCATION_FAILED", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/WALK_TO_LEAK_LOCATION_EXECUTING", FSM_NEXT(STATE_WALK_TO_LEAK_LOCATION))
        FSM_ON_EVENT("/REACHED_LEAK_LOCATION", FSM_NEXT(STATE_LEAK_REPAIR))
      }
    }
    FSM_STATE(STATE_LEAK_REPAIR)
    {

      FSM_CALL_TASK(STATE_LEAK_REPAIR)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/LEAK_REPAIR_EXECUTING", FSM_NEXT(STATE_LEAK_REPAIR))
        FSM_ON_EVENT("/LEAK_REPAIR_FAILED", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/LEAK_REPAIRED", FSM_NEXT(STATE_DETECT_FINISH))
      }
    }
    FSM_STATE(STATE_DETECT_FINISH)
    {

      FSM_CALL_TASK(STATE_DETECT_FINISH)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/DETECT_FINISH_RETRY", FSM_NEXT(STATE_DETECT_FINISH))
        FSM_ON_EVENT("/DETECT_FINISH_FAILED", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/DETECT_FINISH_SUCESSFUL", FSM_NEXT(STATE_WALK_TO_FINISH))
      }
    }
    FSM_STATE(STATE_WALK_TO_FINISH)
    {

      FSM_CALL_TASK(STATE_WALK_TO_FINISH)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/WALK_TO_FINISH_RETRY", FSM_NEXT(STATE_DETECT_FINISH))
        FSM_ON_EVENT("/WALK_TO_FINISH_EXECUTING", FSM_NEXT(STATE_WALK_TO_FINISH))
        FSM_ON_EVENT("/WALK_TO_FINISH_ERROR", FSM_NEXT(STATE_ERROR))
        FSM_ON_EVENT("/WALK_TO_FINISH_SUCESSFUL", FSM_NEXT(STATE_END))
      }
    }
    FSM_STATE(STATE_END)
    {

      FSM_CALL_TASK(STATE_END)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/END_SUCESSFUL", FSM_NEXT(STATE_END))
      }
    }
    FSM_STATE(STATE_ERROR)
    {

      FSM_CALL_TASK(STATE_ERROR)

      FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/RESTART", FSM_NEXT(STATE_INIT))
      }
    }
  }
  FSM_END
}