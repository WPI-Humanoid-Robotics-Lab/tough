#include <src_task1/val_task1_node.h>

#define foreach BOOST_FOREACH

task1Node::task1Node(ros::NodeHandle nh):
    nh_(nh)
{
    task1_ = valTask1::getValTask1(nh);
}

task1Node::~task1Node()
{
    ROS_INFO("task1Node::Destrutor called");
    if (task1_ != nullptr) delete task1_;
}

void task1Node::registerStateMethods(void)
{
    // Register all the functions
    // register the api's for states
    LocalTasks::registrate("STATE_PRE_INIT",            std::bind(&valTask1::pre_initTask,              task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_INIT",                std::bind(&valTask1::initTask,              task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_PANEL_COARSE", std::bind(&valTask1::detectPanelCoarseTask, task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_SEE_PANEL",   std::bind(&valTask1::walkToSeePanelTask,    task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_HANDLE_CENTER",std::bind(&valTask1::detectHandleCenterTask,task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_PANEL_FINE",   std::bind(&valTask1::detectPanelFineTask,   task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_PANEL",       std::bind(&valTask1::walkToPanel,           task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_FIX_HANDLE_DETECTION",std::bind(&valTask1::fixHandle,             task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_GRASP_PITCH_HANDLE",  std::bind(&valTask1::graspPitchHandleTask,  task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_CORRECT_PITCH",       std::bind(&valTask1::controlPitchTask,      task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_GRASP_YAW_HANDLE",    std::bind(&valTask1::graspYawHandleTask,    task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_CORRECT_YAW",         std::bind(&valTask1::controlYawTask,        task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_REDETECT_HANDLE",     std::bind(&valTask1::redetectHandleTask,    task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_FINISH",       std::bind(&valTask1::detectfinishBoxTask,   task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_FINISH",      std::bind(&valTask1::walkToFinishTask,      task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("END_STATE",                 std::bind(&valTask1::endTask,               task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ERROR",               std::bind(&valTask1::errorTask,             task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_SKIP_TO_CP3",         std::bind(&valTask1::skipToCP3,             task1_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void task1Node::paramUpdateCallback(src_task1::task1_parametersConfig &config, uint32_t level)
{
    ROS_INFO("update the goal, x: %f, y: %f, theta: %f",  config.groups.panelwalkpose.x_pw,  config.groups.panelwalkpose.y_pw,  config.groups.panelwalkpose.theta_pw);

    geometry_msgs::Pose2D goal;
    goal.x = config.groups.panelwalkpose.x_pw;
    goal.y = config.groups.panelwalkpose.y_pw;
    goal.theta = config.groups.panelwalkpose.theta_pw;
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

    dynamic_reconfigure::Server<src_task1::task1_parametersConfig> service;
    dynamic_reconfigure::Server<src_task1::task1_parametersConfig>::CallbackType callback_type;
    callback_type = std::bind(&task1Node::paramUpdateCallback, &task1Node, std::placeholders::_1, std::placeholders::_2);
    service.setCallback(callback_type);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting Task 1");
    Fsmsrc_task1(NULL, q, "src_task1");

    spinner.stop();

    return 0;
}
