#include <val_task2/val_task2_node.h>

using namespace std;

#define foreach BOOST_FOREACH


task2Node::task2Node(ros::NodeHandle nh):
    nh_(nh)
{
    task2_ = new valTask2(nh);
}

task2Node::~task2Node()
{

}

void task2Node::registerStateMethods(void)
{
    // Register all the functions
    LocalTasks::registrate("STATE_INIT", &valTask2::initTask);
    LocalTasks::registrate("STATE_DETECT_ROVER", &valTask2::detectRoverTask);
    LocalTasks::registrate("STATE_WALK_TO_ROVER", &valTask2::walkToRoverTask);
    LocalTasks::registrate("STATE_DETECT_SOLAR_PANEL",&valTask2::detectPanelTask);
    LocalTasks::registrate("STATE_ORIENT_TO_SOLAR_PANEL",&valTask2::orientPanelTask);
    LocalTasks::registrate("STATE_PICK_SOLAR_PANEL",&valTask2::pickPanelTask);
    LocalTasks::registrate("STATE_DETECT_SOLAR_ARRAY", &valTask2::detectSolarArrayTask);
    LocalTasks::registrate("STATE_WALK_TO_SOLAR_ARRAY", &valTask2::walkSolarArrayTask);
    LocalTasks::registrate("STATE_PLACE_SOLAR_PANEL_ON_GROUND", &valTask2::placePanelTask);
    LocalTasks::registrate("STATE_DETECT_DEPLOY_PANEL_BUTTON", &valTask2::detectButtonTask);
    LocalTasks::registrate("STATE_DEPLOY_SOLAR_PANEL", &valTask2::deployPanelTask);
    LocalTasks::registrate("STATE_DETECT_POWER_CABLE", &valTask2::dtectCableTask);
    LocalTasks::registrate("STATE_PICKUP_POWER_CABLE", &valTask2::pickCableTask);
    LocalTasks::registrate("STATE_PLUGIN_POWER_CABLE", &valTask2::plugCableTask);
    LocalTasks::registrate("STATE_DETECT_FINISH", &valTask2::detectfinishBoxTask);
    LocalTasks::registrate("STATE_WALK_TO_FINISH", &valTask2::walkToFinishTask);
    LocalTasks::registrate("STATE_END", &valTask2::endTask);
    LocalTasks::registrate("STATE_ERROR", &valTask2::errorTask);
}

void task2Node::paramUpdateCallback(val_task2::task2_parametersConfig &config, uint32_t level)
{
    ROS_INFO("update the goal, x: %f, y: %f, theta: %f",  config.groups.panelwalkpose.x,  config.groups.panelwalkpose.y,  config.groups.panelwalkpose.theta);

    geometry_msgs::Pose2D goal;
    goal.x = config.groups.panelwalkpose.x;
    goal.y = config.groups.panelwalkpose.y;
    goal.theta = config.groups.panelwalkpose.theta;
    //update the walk goal location
    task2_->setPanelWalkGoal(goal);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "task2");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nh;
    RosEventQueue*q = new RosEventQueue();

    task2Node task2node(nh);

    ROS_INFO("Preparing task2...");

    // register the state execution methods
    task2node.registerStateMethods();

    // init synamic reconfigure of the parameters
    //task2node.initDynamicReconfParams();

    dynamic_reconfigure::Server<val_task2::task2_parametersConfig> service;
    dynamic_reconfigure::Server<val_task2::task2_parametersConfig>::CallbackType callback_type;
    callback_type = boost::bind(&task2Node::paramUpdateCallback, task2node, _1, _2);
    service.setCallback(callback_type);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting Task 2");
    Fsmval_task2(NULL, q, "val_task2");

    spinner.stop();

    return 0;
}
