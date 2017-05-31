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

    reset_pointcloud_pub    = nh_.advertise<std_msgs::Empty>("/field/reset_pointcloud",1);
    pause_pointcloud_pub    = nh_.advertise<std_msgs::Bool>("/field/pause_pointcloud",1);
    clearbox_pointcloud_pub = nh_.advertise<std_msgs::Empty>("/field/clearbox_pointcloud",1);

    task_status_sub_        = nh_.subscribe("/srcsim/finals/task", 10, &task2Utils::taskStatusCB, this);

    reOrientPanelTraj_.resize(2);
    reOrientPanelTraj_[0].arm_pose = {-1.2, -1.04, 2.11, -0.85, -1.1, 0, 0.29};
    reOrientPanelTraj_[0].time = 1;

    reOrientPanelTraj_[1].arm_pose = {-1.2, -1.04, 2.11, -0.85, 1.21, 0, 0.29};
    reOrientPanelTraj_[1].time = 2;

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
        seed1 = &leftNearChestGrasp_;
        seed2 = &rightSeedNonGraspingHand_;
    }
    else
    {
        seed1 = &rightNearChestGrasp_;
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

    // raise pelvis
    pelvis_controller_->controlPelvisHeight(1.1);
    ros::Duration(2).sleep();

    const std::vector<float> *graspingHandPoseUp, *graspingHandPoseDown, *nonGraspingHandPose2, *nonGraspingHandPose1;

    if(graspingHand == armSide::LEFT){
        graspingHandPoseUp     = &leftPanelPlacementUpPose1_;
        graspingHandPoseDown   = &leftPanelPlacementDownPose1_;
        nonGraspingHandPose1 = &rightPanelPlacementSupport1_;
        nonGraspingHandPose2 = &rightPanelPlacementSupport2_;
        // take non-GraspingHand out
        arm_controller_->moveArmJoint(nonGraspingHand, 3, 0.5);
        ros::Duration(1).sleep();
    }
    else
    {
        graspingHandPoseUp     = &rightPanelPlacementUpPose1_;
        graspingHandPoseDown   = &rightPanelPlacementDownPose1_;
        nonGraspingHandPose1 = &leftPanelPlacementSupport1_;
        nonGraspingHandPose2 = &leftPanelPlacementSupport2_;
        // take non-GraspingHand out
        arm_controller_->moveArmJoint(nonGraspingHand, 3, -0.5);
        ros::Duration(1).sleep();
    }

    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*graspingHandPoseUp);
    arm_controller_->moveArmJoints(graspingHand, armData, 2.0f);
    ros::Duration(2).sleep();

    gripper_controller_->openGripper(graspingHand);
    ros::Duration(0.5).sleep();

    armData.clear();
    armData.push_back(*graspingHandPoseDown);
    arm_controller_->moveArmJoints(graspingHand, armData, 1.0f);
    ros::Duration(1).sleep();

    std::vector<armTrajectory::armJointData> pushPanel;
    pushPanel.resize(2);
    pushPanel[0].side = nonGraspingHand;
    pushPanel[0].arm_pose = *nonGraspingHandPose1;
    pushPanel[0].time = 1;

    pushPanel[1].side = nonGraspingHand;
    pushPanel[1].arm_pose = *nonGraspingHandPose2;
    pushPanel[1].time = 2;

    arm_controller_->moveArmJoints(pushPanel);
    ros::Duration(3).sleep();

}

void task2Utils::reOrientTowardsPanel(geometry_msgs::Pose panelPose){

    size_t nSteps;
    armSide startStep;
    std::vector<float> y_offset;
    std::vector<float> x_offset = {0,0};

    current_state_->transformPose(panelPose,panelPose,VAL_COMMON_NAMES::WORLD_TF,
                                  VAL_COMMON_NAMES::PELVIS_TF);

    double error = panelPose.position.y;
    if (std::abs(error) < 0.1){
        ROS_INFO("reOrientTowardsPanel: Not reorienting, the offset is less than 0.1");
    }

    else if (std::abs(error) > 0.49){
        ROS_INFO("reOrientTowardsPanel: Offset more than 0.5");
    }

    else{
        nSteps = (std::abs(error))/0.1;

        if (error > 0){
            startStep = LEFT;
            y_offset  = {0.1,0.1};
        }
        else {
            startStep = RIGHT;
            y_offset = {-0.1,-0.1};
        }

        ROS_INFO("reOrientTowardsPanel: Walking %d steps",int(nSteps));

        walk_->walkLocalPreComputedSteps(x_offset,y_offset,startStep);
        ros::Duration(2.0).sleep();

    }

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

void task2Utils::clearPointCloud() {
    std_msgs::Empty msg;
    reset_pointcloud_pub.publish(msg);
    ros::Duration(0.3).sleep();
}

void task2Utils::clearBoxPointCloud() {
    std_msgs::Empty msg;
    clearbox_pointcloud_pub.publish(msg);
}

void task2Utils::pausePointCloud() {
    std_msgs::Bool msg;
    msg.data = true;
    pause_pointcloud_pub.publish(msg);
}

void task2Utils::resumePointCloud() {
    std_msgs::Bool msg;
    msg.data = false;
    pause_pointcloud_pub.publish(msg);
}

