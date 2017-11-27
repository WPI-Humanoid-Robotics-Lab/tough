#include <src_task2/cable_task.h>
#include <tough_controller_interface/gripper_control_interface.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "choke_grabber");
    ros::NodeHandle nh;
    CableTask cg(nh);
    ROS_INFO("Starting cable grabber");
    gripperControl gc(nh);

    if(argc == 9){
        geometry_msgs::Pose pt;
        pt.position.x = std::atof(argv[2]);
        pt.position.y = std::atof(argv[3]);
        pt.position.z = std::atof(argv[4]);
        pt.orientation.x = std::atof(argv[5]);
        pt.orientation.y = std::atof(argv[6]);
        pt.orientation.z = std::atof(argv[7]);
        pt.orientation.w = std::atof(argv[8]);


        RobotSide side;
        if(std::atoi(argv[1]) == 0){
            side = LEFT;
        } else {
            side = RIGHT;
        }
        gc.openGripper(side);
        cg.grasp_choke(side,pt);
    } else{
        ROS_INFO("Usage : %s <side> <goal_x> <goal_y> <goal_z>\n side = 0 or 1");
        return -1;
    }

    ros::spinOnce();
    ros::Duration(2).sleep();
    return 0;
}




