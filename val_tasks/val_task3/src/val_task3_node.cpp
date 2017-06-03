
#include <val_task3/val_task3_node.h>

task3Node::task3Node(ros::NodeHandle nh): nh_(nh)
{
  task3_ = valTask3::getValTask3(nh);
}

task3Node::~task3Node()
{

}

void task3Node::registerStateMethods()
{
  // Register all the functions
  // register the api's for states
  LocalTasks::registrate("STATE_INIT",                         std::bind(&valTask3::initTask,             task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_DETECT_STAIRS",                std::bind(&valTask3::detectStairsTask,     task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_CLIMB_STAIRS",                 std::bind(&valTask3::climbStairsTask,      task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_DETECT_DOOR_HANDLE",           std::bind(&valTask3::detectDoorHandleTask, task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_OPEN_DOOR",                    std::bind(&valTask3::openDoorTask,         task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_WALK_THROUGH_DOOR",            std::bind(&valTask3::walkthroughDoorTask,  task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_DETECT_LEAK_TOOL",             std::bind(&valTask3::detectLeakToolTask,   task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_WALK_TO_LEAK_TOOL",            std::bind(&valTask3::walkToLeakToolTask,   task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_PICK_LEAK_TOOL",               std::bind(&valTask3::pickLeakTool,         task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_WALK_TO_DETECT_LEAK_LOCATION", std::bind(&valTask3::walkToDetectLeak,     task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_DETECT_LEAK_LOCATION",         std::bind(&valTask3::detectLeakTask,       task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_DETECT_REPAIR_TOOL",           std::bind(&valTask3::detectRepairToolTask, task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_WALK_TO_REPAIR_TOOL",          std::bind(&valTask3::walkToRepairToolTask, task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_PICK_REPAIR_TOOL",             std::bind(&valTask3::pickRepairTool,       task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_WALK_TO_LEAK_LOCATION",        std::bind(&valTask3::walkToLeakTask,       task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_LEAK_REPAIR",                  std::bind(&valTask3::leakRepairTask,       task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_WALK_TO_TABLE",                std::bind(&valTask3::walkToTableTask,      task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_DETECT_FINISH",                std::bind(&valTask3::detectFinishTask,     task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_WALK_TO_FINISH",               std::bind(&valTask3::walkToFinishTask,     task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_END",                          std::bind(&valTask3::endTask,              task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  LocalTasks::registrate("STATE_ERROR",                        std::bind(&valTask3::errorTask,            task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}


int main(int argc, char** argv){

  ros::init(argc, argv, "task3");
  ros_decision_making_init(argc, argv);
  ros::NodeHandle nh;
  RosEventQueue *q = new RosEventQueue();

  task3Node task3Node(nh);
  ROS_INFO("Preparing task3");

  // register the state execution methods
  task3Node.registerStateMethods();

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Starting Task 3");
  Fsmval_task3(NULL, q, "val_task3");

  spinner.stop();

  return 0;

}
