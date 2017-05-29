#include <val_task2/button_press.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "button_press_node");
    ros::NodeHandle nh;
    button_press bp_(nh);
    geometry_msgs::Point goal;

    armSide side;
    if(argc == 2)
    {
        if(std::atoi(argv[1]) == 0)
        {
            side = LEFT;
        }
        else
        {
            side = RIGHT;
        }
    }
    else
    {
        ROS_INFO("Usage : %s <side> side = 0 or 1");
        return -1;
    }

    // detecting button location which would act as goal to press the button
//    bp_.getButtonPosition(goal);
    goal.x=-0.413;
    goal.y=-0.0753;
    goal.z=1.24;
    // pressing the button
    bp_.pressButton(side, goal);
    return 0;
}



