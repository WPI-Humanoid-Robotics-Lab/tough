#include <val_task1/val_task1_node.h>

using namespace std;

#define foreach BOOST_FOREACH


task1Node::task1Node(ros::NodeHandle nh):
    nh_(nh)
{
    task1_ = new valTask1(nh);
}

task1Node::~task1Node()
{

}

void task1Node::registerStateMethods(void)
{
    // Register all the functions
    // register the api's for states
    LocalTasks::registrate("STATE_INIT", &valTask1::initTask);
    LocalTasks::registrate("STATE_DETECT_PANEL",  &valTask1::detectPanelTask);
    LocalTasks::registrate("STATE_WALK_TO_CONTROL",  &valTask1::walkToControlPanelTask);
    LocalTasks::registrate("STATE_DETECT_HANDLE_CENTER",  &valTask1::detectHandleCenterTask);
    LocalTasks::registrate("STATE_ADJUST_ARMS",  &valTask1::adjustArmTask);
    LocalTasks::registrate("STATE_CORRECT_PITCH",  &valTask1::controlPitchTask);
    LocalTasks::registrate("STATE_CORRECT_YAW",  &valTask1::controlYawTask);
    LocalTasks::registrate("STATE_DETECT_FINISH",  &valTask1::detectfinishBoxTask);
    LocalTasks::registrate("STATE_WALK_TO_FINISH",  &valTask1::walkToFinishTask);
    LocalTasks::registrate("END_STATE",  &valTask1::endTask);
    LocalTasks::registrate("STATE_ERROR",  &valTask1::errorTask);
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
