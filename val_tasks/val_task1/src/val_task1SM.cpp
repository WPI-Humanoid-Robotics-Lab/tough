#include <val_task1/val_task1.h>

// finite state machine name for tackling task-1
FSM(valTask1)
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


bool preemptiveWait(double ms, decision_making::EventQueue& queue) {
  ROS_INFO("prempting");
  for (int i = 0; i < 100 && !queue.isTerminated(); i++)
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms / 100.0));

  return queue.isTerminated();
}

// state machine state executions
decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  // the state transition can happen from an event externally or can be geenerated here
  //!!!!! depends on the developer and use case

  // generate the event
  //eventQueue.riseEvent("/INIT_SUCESSUFL");

  // wait infinetly until an external even occurs
  while(!preemptiveWait(1000, eventQueue)){
     ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}


decision_making::TaskResult walkToControlPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/REACHED_PANEL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult controlPitchTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/PITCH_CORRECTION_SUCESSFUL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult controlYawTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/YAW_CORRECTION_SUCESSFUL");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult walkToEndTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/WALK_TO_END");
  return TaskResult::SUCCESS();
}

decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
  ROS_INFO_STREAM("executing " << name);

  eventQueue.riseEvent("/STOP_TIMEOUT");
  return TaskResult::SUCCESS();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task1");
  ros_decision_making_init(argc, argv);
  ros::NodeHandle nh;
  RosEventQueue* q = new RosEventQueue();

  ROS_INFO("Preparing Task1...");

  // register the api's for states
  LocalTasks::registrate("STATE_INIT", initTask);
  LocalTasks::registrate("STATE_WALK_TO_CONTROL", walkToControlPanelTask);
  LocalTasks::registrate("STATE_CORRECT_PITCH", controlPitchTask);
  LocalTasks::registrate("STATE_CORRECT_YAW", controlYawTask);
  LocalTasks::registrate("STATE_WALK_TO_FINISH", walkToEndTask);
  LocalTasks::registrate("END_STATE", endTask);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Starting Task1");
  FsmvalTask1(NULL, q, "valTask1");

  spinner.stop();

  return 0;
}
