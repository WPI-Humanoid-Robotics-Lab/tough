
#include <val_task2/insert_cable.h>
#include <val_control/val_gripper_control.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "insert_cable_node");
    ros::NodeHandle nh;
    insertCable ic(nh);
    ROS_INFO("Starting insert cable node");
    gripperControl gc(nh);

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
        gc.openGripper(side);
        ic.insert_cable(side, pt);
    } else{
        ROS_INFO("Usage : %s <side> <goal_x> <goal_y> <goal_z>\n side = 0 or 1");
        return -1;
    }

    ros::spin();
    return 0;
}



