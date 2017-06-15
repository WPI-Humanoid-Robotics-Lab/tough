#include <val_task2/val_task2_node.h>

using namespace std;

#define foreach BOOST_FOREACH


task2Node::task2Node(ros::NodeHandle nh):
    nh_(nh)
{
    task2_ = valTask2::getValTask2(nh);
}

task2Node::~task2Node()
{

}

void task2Node::registerStateMethods(void)
{
    // Register all the functions
    LocalTasks::registrate("STATE_PRE_INIT",                       std::bind(&valTask2::pre_initTask,                   task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_INIT",                       std::bind(&valTask2::initTask,                   task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_ROVER",               std::bind(&valTask2::detectRoverTask,            task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_ROVER",              std::bind(&valTask2::walkToRoverTask,            task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_SOLAR_PANEL",         std::bind(&valTask2::detectPanelTask,            task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_GRASP_SOLAR_PANEL",          std::bind(&valTask2::graspPanelTask,             task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PICK_SOLAR_PANEL",           std::bind(&valTask2::pickPanelTask,              task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_SOLAR_ARRAY",         std::bind(&valTask2::detectSolarArrayTask,       task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_SOLAR_ARRAY",        std::bind(&valTask2::walkSolarArrayTask,         task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ROTATE_PANEL",               std::bind(&valTask2::rotatePanelTask,            task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_FIND_CABLE_INTERMEDIATE",    std::bind(&valTask2::findCableIntermediateTask,  task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_SOLAR_ARRAY_FINE",    std::bind(&valTask2::detectSolarArrayFineTask,   task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ALIGN_TO_SOLAR_ARRAY",       std::bind(&valTask2::alignSolarArrayTask,        task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PLACE_SOLAR_PANEL_ON_GROUND",std::bind(&valTask2::placePanelTask,             task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_DEPLOY_PANEL_BUTTON", std::bind(&valTask2::detectButtonTask,           task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DEPLOY_SOLAR_PANEL",         std::bind(&valTask2::deployPanelTask,            task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_POWER_CABLE",         std::bind(&valTask2::detectCableTask,            task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PICKUP_POWER_CABLE",         std::bind(&valTask2::pickCableTask,              task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_SOCKET",              std::bind(&valTask2::detectSocketTask,           task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_PLUGIN_POWER_CABLE",         std::bind(&valTask2::plugCableTask,              task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_DETECT_FINISH",              std::bind(&valTask2::detectFinishBoxTask,        task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_WALK_TO_FINISH",             std::bind(&valTask2::walkToFinishTask,           task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_END",                        std::bind(&valTask2::endTask,                    task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ERROR",                      std::bind(&valTask2::errorTask,                  task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_SKIP_CHECKPOINT",            std::bind(&valTask2::skipCheckPointTask,         task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_SKIP_TO_CP_3",               std::bind(&valTask2::skipToCP3Task,              task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_SKIP_TO_CP_4",               std::bind(&valTask2::skipToCP4Task,              task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_SKIP_TO_CP_6",               std::bind(&valTask2::skipToCP6Task,              task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_MANUAL_EXECUTION",           std::bind(&valTask2::manualExecutionTask,        task2_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
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
