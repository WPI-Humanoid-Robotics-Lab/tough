
#include <val_task3/val_task3_node.h>

task3Node::task3Node(ros::NodeHandle nh): nh_(nh){
    task3_ = valTask3::getValTask3(nh);
}

task3Node::~task3Node(){

}

void task3Node::registerStateMethods(){

    LocalTasks::registrate("STATE_INIT",      std::bind(&valTask3::initTask,    task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_END",       std::bind(&valTask3::endTask,     task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    LocalTasks::registrate("STATE_ERROR",     std::bind(&valTask3::errorTask,   task3_, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}


int main(int argc, char** argv){

    ros::init(argc, argv, "task3");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nh;
    RosEventQueue *q = new RosEventQueue();

    task3Node task3Node(nh);
    ROS_INFO("Preparing task3");

    task3Node.registerStateMethods();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Starting Task 3");
    Fsmval_task3(NULL, q, "val_task3");

    spinner.stop();

    return 0;

}
