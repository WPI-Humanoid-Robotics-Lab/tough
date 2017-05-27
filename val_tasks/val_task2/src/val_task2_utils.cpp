#include <val_task2/val_task2_utils.h>
#include <std_msgs/Bool.h>

task2Utils::task2Utils(ros::NodeHandle nh):
    nh_(nh)
{
    // controllers
    chest_controller_    = new chestTrajectory(nh_);
    pelvis_controller_   = new pelvisTrajectory(nh_);
    head_controller_     = new HeadTrajectory(nh_);
    gripper_controller_  = new gripperControl(nh_);
    arm_controller_      = new armTrajectory(nh_);
    walk_                = new ValkyrieWalker(nh_, 0.7, 0.7, 0, 0.18);
    current_state_       = RobotStateInformer::getRobotStateInformer(nh_);

    reset_pointcloud_pub = nh_.advertise<std_msgs::Empty>("/field/reset_pointcloud",1);
    pause_pointcloud_pub = nh_.advertise<std_msgs::Bool>("/field/pause_pointcloud",1);
}

task2Utils::~task2Utils()
{
    delete chest_controller_    ;
    delete pelvis_controller_   ;
    delete head_controller_     ;
    delete gripper_controller_  ;
    delete arm_controller_      ;
    delete walk_                ;
}

void task2Utils::afterPanelGraspPose(const armSide side)
{
    // reorient the chest
    chest_controller_->controlChest(0,0,0);
    ros::Duration(2).sleep();


    const std::vector<float> *seed1,*seed2;
    if(side == armSide::LEFT){
        seed1 = &leftSeedGraspingHand_;
        seed2 = &rightSeedNonGraspingHand_;
    }
    else
    {
        seed1 = &rightSeedGraspingHand;
        seed2 = &leftSeedNonGraspingHand_;
    }

    std::vector< std::vector<float> > armData;

    armData.clear();
    armData.push_back(*seed2);
    arm_controller_->moveArmJoints((armSide)(!side), armData, 2.0f);
    ros::Duration(0.5).sleep();

    armData.clear();
    armData.push_back(*seed1);
    arm_controller_->moveArmJoints(side, armData, 2.0f);
    ros::Duration(2).sleep();
}

void task2Utils::movePanelToWalkSafePose(const armSide side)
{
    // reorient the chest
    chest_controller_->controlChest(0,0,0);
    ros::Duration(2).sleep();

    const std::vector<float> *seed1,*seed2;
    if(side == armSide::LEFT){
        seed2 = &leftShoulderSeedPanelGraspWalk_;
    }
    else
    {
        seed2 = &rightShoulderSeedPanelGraspWalk_;
    }
    std::vector<double> grasp = {1.2, -0.6, -0.77, -0.9, -0.9};
    gripper_controller_->controlGripper(side, grasp);
    ros::Duration(1).sleep();
    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*seed2);

    arm_controller_->moveArmJoints(side, armData, 2.0f);
    ros::Duration(2).sleep();

}

//threshold is selected by experimentation
#define THRESHOLD 55
bool task2Utils::isPanelPicked(const armSide side)
{
    std::string jointNames = side == armSide::LEFT ? "left_arm" : "right_arm";
    std::vector<float> jointEfforts;
    current_state_->getJointEfforts(jointNames,jointEfforts);
    float total_effort=0.0f;
    for (float effort : jointEfforts){
        total_effort += fabs(effort);
    }
    ROS_INFO("Total effort on arm is %f", total_effort);

    if (total_effort > THRESHOLD){
        return true;
    }

    return false;
}

void task2Utils::clearPointCloud()
{
    std_msgs::Empty msg;
    reset_pointcloud_pub.publish(msg);
}

void task2Utils::pausePointCloud()
{
    std_msgs::Bool msg;
    msg.data = true;
    pause_pointcloud_pub.publish(msg);
}

void task2Utils::resumePointCloud()
{
    std_msgs::Bool msg;
    msg.data = false;
    pause_pointcloud_pub.publish(msg);
}
