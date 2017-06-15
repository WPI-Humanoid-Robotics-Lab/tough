#include <ros/ros.h>
//#include <val_footstep/ValkyrieWalker.h>
#include "val_controllers/robot_state.h"
#include "std_msgs/String.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "check_goal_offset");
    ros::NodeHandle nh;
    RobotStateInformer *current_state_;
    current_state_ = RobotStateInformer::getRobotStateInformer(nh);
    ros::Publisher tasklogPB = nh.advertise<std_msgs::String>("/field/log",10);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Duration(2).sleep();
    ros::spinOnce();

    geometry_msgs::Pose pelvisPose;
    geometry_msgs::Pose pelvisPoseinPelvis;



    if(argc == 8){
        geometry_msgs::Pose pt;
        pt.position.x = std::atof(argv[1]);
        pt.position.y = std::atof(argv[2]);
        pt.position.z = std::atof(argv[3]);
        pt.orientation.x = std::atof(argv[4]);
        pt.orientation.y = std::atof(argv[5]);
        pt.orientation.z = std::atof(argv[6]);
        pt.orientation.w = std::atof(argv[7]);

        current_state_->getCurrentPose("/pelvis",pelvisPose);
        ros::Duration(1.0).sleep();
        current_state_->getCurrentPose("/pelvis",pelvisPoseinPelvis,VAL_COMMON_NAMES::PELVIS_TF);
        ros::spinOnce();

        float x_offset =pelvisPose.position.x -pt.position.x;
        float y_offset =pelvisPose.position.y -pt.position.y;
        float z_offset =pelvisPose.position.z -pt.position.z;
        float w =pelvisPose.orientation.w -pt.orientation.w;
        float x =pelvisPose.orientation.x -pt.orientation.x;
        float y =pelvisPose.orientation.y -pt.orientation.y;
        float z =pelvisPose.orientation.z -pt.orientation.z;

        std::cout<<"x offset in world frame: "<<x_offset<<"\n";
        std::cout<<"y offset in world frame: "<<y_offset<<"\n";
        std::cout<<"z offset in world frame: "<<z_offset<<"\n";
        std::cout<<"w offset in world frame: "<<w<<"\n";
        std::cout<<"x offset in world frame: "<<x<<"\n";
        std::cout<<"y offset in world frame: "<<y<<"\n";
        std::cout<<"z offset in world frame: "<<z<<"\n";

        current_state_->transformPose(pt,pt,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);

        x_offset =pelvisPoseinPelvis.position.x -pt.position.x;
        y_offset =pelvisPoseinPelvis.position.y -pt.position.y;
        z_offset =pelvisPoseinPelvis.position.z -pt.position.z;
        w =pelvisPoseinPelvis.orientation.w -pt.orientation.w;
        x =pelvisPoseinPelvis.orientation.x -pt.orientation.x;
        y =pelvisPoseinPelvis.orientation.y -pt.orientation.y;
        z =pelvisPoseinPelvis.orientation.z -pt.orientation.z;

        std::cout<<"x offset in pelvis frame: "<<x_offset<<"\n";
        std::cout<<"y offset in pelvis frame: "<<y_offset<<"\n";
        std::cout<<"z offset in pelvis frame: "<<z_offset<<"\n";
        std::cout<<"w offset in pelvis frame: "<<w<<"\n";
        std::cout<<"x offset in pelvis frame: "<<x<<"\n";
        std::cout<<"y offset in pelvis frame: "<<y<<"\n";
        std::cout<<"z offset in pelvis frame: "<<z<<"\n";


    }
    else if(argc == 4)
    {
        geometry_msgs::Point pt;
        pt.x = std::atof(argv[1]);
        pt.y = std::atof(argv[2]);
        pt.z = std::atof(argv[3]);

        current_state_->getCurrentPose("/pelvis",pelvisPose);
        ros::Duration(1.0).sleep();
        current_state_->getCurrentPose("/pelvis",pelvisPoseinPelvis,VAL_COMMON_NAMES::PELVIS_TF);
        ros::spinOnce();

        float x_offset =pelvisPose.position.x -pt.x;
        float y_offset =pelvisPose.position.y -pt.y;
        float z_offset =pelvisPose.position.z -pt.z;

        std::cout<<"x offset in world frame: "<<x_offset<<"\n";
        std::cout<<"y offset in world frame: "<<y_offset<<"\n";
        std::cout<<"z offset in world frame: "<<z_offset<<"\n";

        current_state_->transformPoint(pt,pt,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);

        x_offset =pelvisPoseinPelvis.position.x -pt.x;
        y_offset =pelvisPoseinPelvis.position.y -pt.y;
        z_offset =pelvisPoseinPelvis.position.z -pt.z;

        std::cout<<"x offset in pelvis frame: "<<x_offset<<"\n";
        std::cout<<"y offset in pelvis frame: "<<y_offset<<"\n";
        std::cout<<"z offset in pelvis frame: "<<z_offset<<"\n";
    }

    return 0;
}
