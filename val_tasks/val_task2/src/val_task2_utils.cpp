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

    current_checkpoint_  = 0;

    reset_pointcloud_pub = nh_.advertise<std_msgs::Empty>("/field/reset_pointcloud",1);
    pause_pointcloud_pub = nh_.advertise<std_msgs::Bool>("/field/pause_pointcloud",1);

    task_status_sub_     = nh_.subscribe("/srcsim/finals/task", 10, &task2Utils::taskStatusCB, this);
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
    const std::vector<double> *grasp;
    if(side == armSide::LEFT){
        //        when left hand is the provided side, we move right hand under the panel
        seed2 = &rightShoulderSeedPanelGraspWalk_;
        grasp = &leftHandGrasp_;
    }
    else
    {
        //        when right hand is the provided side, we move left hand under the panel
        seed2 = &leftShoulderSeedPanelGraspWalk_;
        grasp = &rightHandGrasp_;
    }

    gripper_controller_->controlGripper(side, *grasp);
    ros::Duration(1).sleep();

    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*seed2);

    arm_controller_->moveArmJoints((armSide)!side, armData, 2.0f);
    ros::Duration(2).sleep();

}

#define EFFORT_THRESHOLD 55 //threshold is selected by experimentation
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

    if (total_effort > EFFORT_THRESHOLD){
        return true;
    }

    return false;
}

void task2Utils::moveToPlacePanelPose(const armSide graspingHand, bool rotatePanel)
{
    if (rotatePanel) {
        return moveToPlacePanelPose2(graspingHand);
    }

    armSide nonGraspingHand = (armSide) !graspingHand;
    // take non-GraspingHand out
//    arm_controller_->moveToZeroPose(nonGraspingHand);

    // raise pelvis
    pelvis_controller_->controlPelvisHeight(1.1);
    ros::Duration(2).sleep();

    const std::vector<float> *graspingHandPose, *nonGraspingHandPose;

    if(graspingHand == armSide::LEFT){
        graspingHandPose    = &leftPanelPlacementPose1_;
        nonGraspingHandPose = &rightPanelPlacementSupport_;
        // take non-GraspingHand out
        arm_controller_->moveArmJoint(nonGraspingHand, 3, 0.5);
        ros::Duration(1).sleep();
    }
    else
    {
        graspingHandPose    = &rightPanelPlacementPose1_;
        nonGraspingHandPose = &leftPanelPlacementSupport_;
        // take non-GraspingHand out
        arm_controller_->moveArmJoint(nonGraspingHand, 3, -0.5);
        ros::Duration(1).sleep();
    }

    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*graspingHandPose);
    arm_controller_->moveArmJoints(graspingHand, armData, 2.0f);
    ros::Duration(2).sleep();

    gripper_controller_->openGripper(graspingHand);
    ros::Duration(0.5).sleep();

    //{-1.3, -1.4, 1.39, -0.9, -1.10, 0.5, 0.3};
    armData.clear();
    armData.push_back({-1.2, -1.4, 1.39, -0.9, -1.10, 0.5, 0.4});
    arm_controller_->moveArmJoints(graspingHand, armData, 1.0f);
    ros::Duration(1).sleep();

    armData.clear();
    armData.push_back(*nonGraspingHandPose);
    arm_controller_->moveArmJoints(nonGraspingHand, armData, 2.0f);
    ros::Duration(2).sleep();

}


int task2Utils::getCurrentCheckpoint() const{
    return current_checkpoint_;
}


void task2Utils::moveToPlacePanelPose2(const armSide graspingHand){

}

void task2Utils::taskStatusCB(const srcsim::Task &msg)
{
    if (msg.current_checkpoint != current_checkpoint_){
        current_checkpoint_ = msg.current_checkpoint;
        ROS_INFO("task2Utils::taskStatusCB : Current checkpoint : %d", current_checkpoint_);
    }

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

