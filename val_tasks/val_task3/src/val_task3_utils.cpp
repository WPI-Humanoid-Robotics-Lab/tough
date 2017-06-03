#include <val_task3/val_task3_utils.h>
#include <math.h>

task3Utils::task3Utils(ros::NodeHandle nh): nh_(nh),arm_controller_(nh_) {
    visited_map_sub_  = nh_.subscribe("/visited_map",10, &task3Utils::visited_map_cb, this);
}

task3Utils::~task3Utils(){

}

void task3Utils::beforePanelManipPose(){

   std::vector< std::vector<float> > armData;
   armData.push_back(RIGHT_ARM_SEED_TABLE_MANIP);

    arm_controller_.moveArmJoints(RIGHT,armData,2.0f);
    ros::Duration(2.0).sleep();

    armData.clear();
    armData.push_back(LEFT_ARM_SEED_TABLE_MANIP);
    arm_controller_.moveArmJoints(LEFT,armData,2.0f);
    ros::Duration(2.0).sleep();
}

void task3Utils::beforDoorOpenPose(){


    std::vector< std::vector<float> > armData;
    armData.push_back(RIGHT_ARM_DOOR_OPEN);

     arm_controller_.moveArmJoints(RIGHT,armData,2.0f);
     ros::Duration(2.0).sleep();

     armData.clear();
     armData.push_back(LEFT_ARM_DOOR_OPEN);
     arm_controller_.moveArmJoints(LEFT,armData,2.0f);
     ros::Duration(2.0).sleep();
}

void task3Utils::visited_map_cb(const nav_msgs::OccupancyGrid::Ptr msg)
{
    visited_map_ = *msg;
}

void task3Utils::blindNavigation(geometry_msgs::Pose2D & goal){
    // get current state
    RobotStateInformer *current_state = RobotStateInformer::getRobotStateInformer(nh_);
    geometry_msgs::Pose poseInPelvisFrame, tempPose;
    current_state->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,poseInPelvisFrame);

    tempPose = poseInPelvisFrame;
    float angle = 0.0;
    int tempX, tempY, radius = 2, angleStep = 30;


    // calculate points in a circle

    for (size_t i = 0; i<7; i++){
        angle = angle + pow(-1,i)*i*angleStep; // start from zero and check alternate directions in increments of 30
        tempX = poseInPelvisFrame.position.x + radius * cos(angle);
        tempY = poseInPelvisFrame.position.y + radius * sin(angle);

        size_t index = MapGenerator::getIndex(tempX, tempY);
        ROS_INFO("Index in map %d and size of visited map is %d", (int)index, (int)visited_map_.data.size());

        if(visited_map_.data.at(index) != CELL_STATUS::FREE){
            continue;
        }
        else
        {
            goal.x = tempX;
            goal.y = tempY;
            goal.theta = angle;
        }
    }

}
