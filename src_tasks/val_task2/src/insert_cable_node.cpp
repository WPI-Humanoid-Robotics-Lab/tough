#include <val_task2/cable_task.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "insert_cable_node");
    ros::NodeHandle nh_;
    CableTask cable(nh_);
    RobotStateInformer *current_state_;
    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    RobotDescription *rd_ = RobotDescription::getRobotDescription(nh_);
    ROS_INFO("Starting insert cable node");
    geometry_msgs::Point pt;
    float offset;
    if(argc == 4){
        pt.x = std::atof(argv[1]);
        pt.y = std::atof(argv[2]);
        pt.z = std::atof(argv[3]);

        cable.insert_cable(pt);
    }
    else if(argc == 5)
    {
        pt.x = std::atof(argv[1]);
        pt.y = std::atof(argv[2]);
        pt.z = std::atof(argv[3]);
        offset = std::atof(argv[4]);

        cable.allign_socket_axis(pt,offset);
    }
    else if(argc == 6)
    {
        pt.x = std::atof(argv[1]);
        pt.y = std::atof(argv[2]);
        pt.z = std::atof(argv[3]);
        offset = std::atof(argv[4]);

        if(std::atoi(argv[5]) == 1)
        {
            current_state_->transformPoint(pt,pt,VAL_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());
            pt.x+=0.03;
            current_state_->transformPoint(pt,pt,rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF);
        }
        else if(std::atoi(argv[5]) == -1)
        {
            current_state_->transformPoint(pt,pt,VAL_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());
            pt.x-=0.03;
            current_state_->transformPoint(pt,pt,rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF);
        }
        else if(std::atoi(argv[5]) == 2)
        {
            current_state_->transformPoint(pt,pt,VAL_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());
            pt.y+=0.03;
            current_state_->transformPoint(pt,pt,rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF);
        }
        else
        {
            current_state_->transformPoint(pt,pt,VAL_COMMON_NAMES::WORLD_TF,rd_->getPelvisFrame());
            pt.y-=0.03;
            current_state_->transformPoint(pt,pt,rd_->getPelvisFrame(),VAL_COMMON_NAMES::WORLD_TF);
        }


        cable.allign_socket_axis(pt,offset);
    }
    else
    {
        ROS_INFO("Usage : %s <side> <goal_x> <goal_y> <goal_z>\n side = 0 or 1");
        return -1;
    }

    ros::spinOnce();
    ros::Duration(2).sleep();
    return 0;
}



