#include <val_task2/cable_task.h>
#include <val_controllers/val_gripper_control.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_cable_node");
    ros::NodeHandle nh;
    CableTask cg(nh);
    ROS_INFO("Starting rotate cable node");
    gripperControl gc(nh);
    geometry_msgs::Pose pt;
    if(argc == 10){
        pt.position.x = std::atof(argv[3]);
        pt.position.y = std::atof(argv[4]);
        pt.position.z = std::atof(argv[5]);
        pt.orientation.x = std::atof(argv[6]);
        pt.orientation.y = std::atof(argv[7]);
        pt.orientation.z = std::atof(argv[8]);
        pt.orientation.w = std::atof(argv[9]);

        armSide side;
        if(std::atoi(argv[1]) == 0){
            side = LEFT;
        } else {
            side = RIGHT;
        }
        gc.openGripper(side);
        if(std::atoi(argv[2]) == 1) cg.rotate_cable1(pt);
        else if(std::atoi(argv[2]) == 2) cg.rotate_cable2(pt);
        else if(std::atoi(argv[2]) == 3) cg.rotate_cable3(pt);

    } else{
        ROS_INFO(" first argument specifies rotate type - 1, 2, 3. Then postion (3 args) and orientation(4 args)");
        return -1;
    }

    ros::spinOnce();
    ros::Duration(2).sleep();
    return 0;
}




