#include <val_task2/button_press.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "button_press_node");
    ros::NodeHandle nh;
    button_press bp_(nh);
    geometry_msgs::Point goal;

    if(argc == 5){
        geometry_msgs::Point pt;
        pt.x = std::atof(argv[2]);
        pt.y = std::atof(argv[3]);
        pt.z = std::atof(argv[4]);

        armSide side;
        if(std::atoi(argv[1]) == 0){
            side = LEFT;
        } else {
            side = RIGHT;
        }
        bp_.pressButton(side, pt);
    } else{
        ROS_INFO("Usage : %s <side> <goal_x> <goal_y> <goal_z>\n side = 0 or 1");
        return -1;
    }

    return 0;
}



