#include <val_task1/val_task1_node.h>

#define foreach BOOST_FOREACH

task1Node::task1Node(ros::NodeHandle nh):
    nh_(nh)
{
    task1_ = valTask1::getValTask1(nh);
}

task1Node::~task1Node()
{

}

void task1Node::registerStateMethods(void)
{
    // Register all the functions
    // register the api's for states
    LocalTasks::registrate("STATE_INIT",                std::bind(&valTask1::initTask,              task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_PANEL_COARSE", std::bind(&valTask1::detectPanelTask,       task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_CONTROL",     std::bind(&valTask1::walkToControlPanelTask,task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_HANDLE_CENTER",std::bind(&valTask1::detectHandleCenterTask,task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ADJUST_ARMS",         std::bind(&valTask1::adjustArmTask,         task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_CORRECT_PITCH",       std::bind(&valTask1::controlPitchTask,      task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_CORRECT_YAW",         std::bind(&valTask1::controlYawTask,        task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_FINISH",       std::bind(&valTask1::detectfinishBoxTask,   task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_FINISH",      std::bind(&valTask1::walkToFinishTask,      task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("END_STATE",                 std::bind(&valTask1::endTask,               task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ERROR",               std::bind(&valTask1::errorTask,             task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void task1Node::paramUpdateCallback(val_task1::task1_parametersConfig &config, uint32_t level)
{
    ROS_INFO("update the goal, x: %f, y: %f, theta: %f",  config.groups.panelwalkpose.x,  config.groups.panelwalkpose.y,  config.groups.panelwalkpose.theta);

    geometry_msgs::Pose2D goal;
    goal.x = config.groups.panelwalkpose.x;
    goal.y = config.groups.panelwalkpose.y;
    goal.theta = config.groups.panelwalkpose.theta;
    //update the walk goal location
    task1_->setPanelWalkGoal(goal);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task1");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nh;
    RosEventQueue*q = new RosEventQueue();

    task1Node task1Node(nh);

    ROS_INFO("Preparing task1...");

    // register the state execution methods
    task1Node.registerStateMethods();

    dynamic_reconfigure::Server<val_task1::task1_parametersConfig> service;
    dynamic_reconfigure::Server<val_task1::task1_parametersConfig>::CallbackType callback_type;
    callback_type = boost::bind(&task1Node::paramUpdateCallback, task1Node, _1, _2);
    service.setCallback(callback_type);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting Task 1");
    Fsmval_task1(NULL, q, "val_task1");

    spinner.stop();

    return 0;
}
