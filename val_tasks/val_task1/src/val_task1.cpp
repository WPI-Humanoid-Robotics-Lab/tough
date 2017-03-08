#include <val_task1/val_task1.h>

// finite state machine name for tackling task-1
FSM(VAL_TASK1)
{
  //define the states
  FSM_STATES
  {
    INIT,
    WALK_TO_SATELLITE_CONTROL,
    CORRECT_PITCH,
    CORRECT_YAW,
    WALK_TO_FINISH,
    END
  }

  // give the start state
  FSM_START(INIT);

  // state machine structure and logic, describe the states tansitions
  FSM_BGN // begin state machine
  {
    FSM_STATE(INIT)
    {
      // state excecution
      FSM_CALL_TASK(INIT)

          // state transitions
          FSM_TRANSITIONS
      {
        // create transitions based on the requirment
        // on event
        FSM_ON_EVENT("/FOUND", FSM_NEXT(WALK_TO_SATELLITE_CONTROL))

            // or on condition or next state directly
      }
    }
    FSM_STATE(WALK_TO_SATELLITE_CONTROL)
    {
      FSM_CALL_TASK(WALK_TO_SATELLITE_CONTROL)

          FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/CLOSE_TO_OBJECT", FSM_NEXT(CORRECT_PITCH))
      }
    }
    FSM_STATE(CORRECT_PITCH)
    {

      FSM_CALL_TASK(CORRECT_PITCH)

          FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/GRABBED", FSM_NEXT(CORRECT_YAW))
      }
    }
    FSM_STATE(CORRECT_YAW)
    {

      FSM_CALL_TASK(CORRECT_YAW)

          FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/GRABBED", FSM_NEXT(WALK_TO_FINISH))
      }
    }
    FSM_STATE(WALK_TO_FINISH)
    {

      FSM_CALL_TASK(WALK_TO_FINISH)

          FSM_TRANSITIONS
      {
        FSM_ON_EVENT("/GRABBED", FSM_NEXT(END))
      }
    }
    FSM_STATE(END)
    {

      FSM_CALL_TASK(END)

          FSM_TRANSITIONS
      {
        //FSM_ON_EVENT("/GRABBED", FSM_NEXT(END))
      }
    }
  }
  FSM_END // end of state machine
}



// state machine state executions
decision_making::TaskResult init(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/init_finish");
  return TaskResult::SUCCESS();
}


decision_making::TaskResult walkToControlPanel(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO("walk to panel");

  eventQueue.riseEvent("/WALK_TIMEOUT");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult controlPitch(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO("control pitch");

  eventQueue.riseEvent("/STOP_TIMEOUT");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult controlYaw(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO("stop...");

  eventQueue.riseEvent("/STOP_TIMEOUT");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult walkToEnd(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO("walk to end");

  eventQueue.riseEvent("/STOP_TIMEOUT");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult End(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO("end");

  eventQueue.riseEvent("/STOP_TIMEOUT");
  return TaskResult::SUCCESS();
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "task1");
  ros_decision_making_init(argc, argv);
  ros::NodeHandle nh;
  RosEventQueue eventQueue;

  // register the api's for states
  LocalTasks::registrate("INIT", init);
  LocalTasks::registrate("WALK_TO_SATELLITE_CONTROL", walkToControlPanel);
  LocalTasks::registrate("CORRECT_PITCH", controlPitch);
  LocalTasks::registrate("CORRECT_YAW", controlYaw);
  LocalTasks::registrate("WALK_TO_FINISH", walkToEnd);
  LocalTasks::registrate("END", End);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ROS_INFO("Starting Task1");
  FsmVAL_TASK1(NULL, &eventQueue);

  spinner.stop();

  return 0;
}
