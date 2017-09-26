#include "val_task2/solar_panel_grasp.h"
#include "tough_controller_interface/gripper_control_interface.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "handle_grabber");
    ros::NodeHandle nh;
    solar_panel_handle_grabber hg(nh);
    ROS_INFO("Starting handle grabber");
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

        armSide side;
        if(std::atoi(argv[1]) == 0){
            side = LEFT;
        } else {
            side = RIGHT;
        }
        gc.openGripper(side);
        hg.grasp_handles(side, pt, false);
    } else{
        ROS_INFO("Usage : %s <side> <goal_x> <goal_y> <goal_z>\n side = 0 or 1");
        return -1;
    }

    ros::spin();
    return 0;
}


