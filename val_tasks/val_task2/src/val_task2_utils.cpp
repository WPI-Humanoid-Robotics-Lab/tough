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
    walk_                = new RobotWalker(nh_, 0.7, 0.7, 0, 0.18);
    current_state_       = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
    cable_detector_      = nullptr;

    wholebody_controller_ = new wholebodyManipulation(nh_);
    right_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::RIGHT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);
    left_arm_planner_ = new cartesianPlanner(VAL_COMMON_NAMES::LEFT_ENDEFFECTOR_GROUP, VAL_COMMON_NAMES::WORLD_TF);


    current_checkpoint_  = 0;
    table_height_        = 0.826; //trial value

    reset_pointcloud_pub    = nh_.advertise<std_msgs::Empty>("/field/reset_pointcloud",1);
    pause_pointcloud_pub    = nh_.advertise<std_msgs::Bool>("/field/pause_pointcloud",1);
    clearbox_pointcloud_pub = nh_.advertise<std_msgs::Int8>("/field/clearbox_pointcloud",1);
    clear_pose_map          = nh_.advertise<std_msgs::Empty>("/field/map/clear_current_pose",1);

    reset_map_pub    = nh_.advertise<std_msgs::Empty>("/field/reset_map",1);
    task2_log_pub_   = nh_.advertise<std_msgs::String>("/field/log",10);

    task_status_sub_        = nh_.subscribe("/srcsim/finals/task", 10, &task2Utils::taskStatusCB, this);
    mapUpdaterSub_          = nh_.subscribe("/map", 10, &task2Utils::mapUpdateCB, this);
    detach_harness          = nh_.subscribe("/srcsim/finals/harness",10, &task2Utils::isDetachedCB, this);


    // rotate panel trajectory for left hand
    reOrientPanelTrajLeft_.resize(2);
    reOrientPanelTrajLeft_[0].arm_pose = {-1.2, -1.04, 2.11, -0.85, -1.15, 0, 0.29};
    reOrientPanelTrajLeft_[0].time = 1;
    reOrientPanelTrajLeft_[0].side = armSide::LEFT;
    reOrientPanelTrajLeft_[1].arm_pose = {-1.2, -1.04, 2.11, -0.85, 1.21, 0, -0.29};
    reOrientPanelTrajLeft_[1].time = 2;
    reOrientPanelTrajLeft_[1].side = armSide::LEFT;

    // rotate panel trajectory for right hand
    reOrientPanelTrajRight_.resize(2);
    reOrientPanelTrajRight_[0].arm_pose = {-1.2, 1.04, 2.11, 0.85, -1.21, 0, -0.29};
    reOrientPanelTrajRight_[0].time = 1;
    reOrientPanelTrajRight_[0].side = armSide::RIGHT;
    reOrientPanelTrajRight_[1].arm_pose = {-1.2, 1.04, 2.11, 0.85, 1.15, 0, 0.29};
    reOrientPanelTrajRight_[1].time = 2;
    reOrientPanelTrajRight_[1].side = armSide::RIGHT;

    isHarnessDetached= false;

    timeNow = boost::posix_time::second_clock::local_time();

    logFile = ros::package::getPath("val_task2") + "/log/task2.csv";
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
void task2Utils::isDetachedCB(const srcsim::Harness &harnessMsg)
{
    mtx_.lock();
    if(harnessMsg.status == 5)
    {
        isHarnessDetached = true;
    }
    mtx_.unlock();
}

bool task2Utils::afterPanelGraspPose(const armSide side, bool isRotationRequired)
{
    // reorienting the chest would bring the panel above the rover
    //    chest_controller_->controlChest(0,0,0);
    //    ros::Duration(2).sleep();

    const std::vector<float> *seed1,*seed2;
    if(side == armSide::LEFT){
        seed1 = isRotationRequired ? &leftNearChestPalmUp_ : &leftNearChestPalmDown_;
        seed2 = &rightSeedNonGraspingHand_;
    }
    else
    {
        seed1 = isRotationRequired ? &rightNearChestPalmUp_ : &rightNearChestPalmDown_;
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

    if (!isPanelPicked(side))
        return false;

    return true;


}

bool task2Utils::isPointOnWalkway(float x, float y)
{
    size_t index = MapGenerator::getIndex(x, y);
    return map_.data.at(index) == CELL_STATUS::FREE;
}

void task2Utils::movePanelToWalkSafePose(const armSide side, bool isRotationRequired)
{
    // reorient the chest
    chest_controller_->controlChest(0,0,0);
    ros::Duration(2).sleep();

    const std::vector<float> *seed1;
    const std::vector<double> *grasp;
    if(side == armSide::LEFT){
        //        when left hand is the provided side, we move right hand under the panel
        seed1 = &rightShoulderSeedPanelGraspWalk_;
        grasp = &leftHandGrasp_;
    }
    else
    {
        //        when right hand is the provided side, we move left hand under the panel
        seed1 = &leftShoulderSeedPanelGraspWalk_;
        grasp = &rightHandGrasp_;
    }
    if(isRotationRequired){
        gripper_controller_->controlGripper(side, *grasp);
        ros::Duration(1).sleep();
    }

    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*seed1);

    arm_controller_->moveArmJoints((armSide)!side, armData, 2.0f);
    ros::Duration(2).sleep();

}

#define EFFORT_THRESHOLD 50 //threshold is selected by experimentation
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
    taskLogPub("valTask2:: Total effort on arm is" + std::to_string(total_effort));

    if (total_effort > EFFORT_THRESHOLD){
        taskLogPub(TEXT_GREEN + "Total effort on arm is "+ std::to_string(total_effort) + TEXT_NC);
        return true;
    }

    taskLogPub(TEXT_RED + "Total effort on arm is "+ std::to_string(total_effort) + TEXT_NC);
    return false;
}

void task2Utils::placePanel(const armSide graspingHand, bool isPanelRotated)
{
    isPanelRotated = true; // this is to avoid rework. I'll fix it the right way when I have time

    armSide nonGraspingHand = (armSide) !graspingHand;

    const std::vector<float> *graspingHandPoseUp, *graspingHandPoseDown;
    const std::vector<float>  *nonGraspingHandPose2, *nonGraspingHandPose1;

    if(graspingHand == armSide::LEFT){
        if (isPanelRotated){
            graspingHandPoseUp     = &leftPanelPlacementUpPose1_;
            graspingHandPoseDown   = &leftPanelPlacementDownPose1_;
        }
        else{
            graspingHandPoseUp     = &leftPanelPlacementUpPose2_;
            graspingHandPoseDown   = &leftPanelPlacementDownPose2_;
        }
        nonGraspingHandPose1 = &rightPanelPlacementSupport1_;
        nonGraspingHandPose2 = &rightPanelPlacementSupport2_;
        // take non-GraspingHand out
//        arm_controller_->moveArmJoint(nonGraspingHand, 3, 0.5);
        ros::Duration(1).sleep();
    }
    else
    {
        if (isPanelRotated){
            graspingHandPoseUp     = &rightPanelPlacementUpPose1_;
            graspingHandPoseDown   = &rightPanelPlacementDownPose1_;
        }
        else {
            graspingHandPoseUp     = &rightPanelPlacementUpPose2_;
            graspingHandPoseDown   = &rightPanelPlacementDownPose2_;
        }
        nonGraspingHandPose1 = &leftPanelPlacementSupport1_;
        nonGraspingHandPose2 = &leftPanelPlacementSupport2_;
        // take non-GraspingHand out
//        arm_controller_->moveArmJoint(nonGraspingHand, 3, -0.5);
//        ros::Duration(1).sleep();
    }

    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*graspingHandPoseUp);
    arm_controller_->moveArmJoints(graspingHand, armData, 2.0f);
    ros::Duration(2).sleep();


    //    if (isPanelPicked(graspingHand)){
    armData.clear();
    armData.push_back(*graspingHandPoseDown);
    arm_controller_->moveArmJoints(graspingHand, armData, 1.0f);
    ros::Duration(1).sleep();
    //    }

    gripper_controller_->openGripper(graspingHand);
    ros::Duration(1).sleep();

    armData.clear();
    armData.push_back(*graspingHandPoseUp);
    arm_controller_->moveArmJoints(graspingHand, armData, 2.0f);
    ros::Duration(2).sleep();


    /* push is not required as we always place the panel with palm down configuration
     *
    std::vector<armTrajectory::armJointData> pushPanel;
    pushPanel.resize(3);
    pushPanel[0].side = nonGraspingHand;
    pushPanel[0].arm_pose = *nonGraspingHandPose1;
    pushPanel[0].time = 1;

    pushPanel[1].side = nonGraspingHand;
    pushPanel[1].arm_pose = *nonGraspingHandPose2;
    pushPanel[1].time = 2;

    pushPanel[2].side = nonGraspingHand;
    pushPanel[2].arm_pose = *nonGraspingHandPose1;
    pushPanel[2].time = 5;

    arm_controller_->moveArmJoints(pushPanel);
     Duration is less than trajectory time as the next step should execute before moving non-grasping hand
    ros::Duration(2.5).sleep();
    */

}

void task2Utils::rotatePanel(const armSide graspingHand)
{
    armSide nonGraspingHand = (armSide) !graspingHand;

    /// @todo change the grip when rotation is perfect
    gripper_controller_->closeGripper(graspingHand);

    const std::vector<float> *graspingHandPoseUp;
    std::vector<armTrajectory::armJointData>* reOrientPanelTraj;
    float tempOffset;
    if(graspingHand == armSide::LEFT){
        //        graspingHandPoseUp = &leftNearChestPalmDown_;
        graspingHandPoseUp = &leftPanelPlacementUpPose1_;
        reOrientPanelTraj = &reOrientPanelTrajLeft_;
        tempOffset = -0.5;
    }
    else
    {
        //        graspingHandPoseUp = &rightNearChestPalmDown_;
        graspingHandPoseUp = &rightPanelPlacementUpPose1_;
        reOrientPanelTraj = &reOrientPanelTrajRight_;
        tempOffset = 0.5;
    }
    // take non-GraspingHand out
    arm_controller_->moveArmJoint(nonGraspingHand, 3, tempOffset);
    ros::Duration(1).sleep();

    arm_controller_->moveArmJoints(*reOrientPanelTraj);
    ros::Duration(3).sleep();

    //    gripper_controller_->controlGripper(graspingHand, GRIPPER_STATE::TIGHT_HOLD);
    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*graspingHandPoseUp);
    arm_controller_->moveArmJoints(graspingHand, armData, 2.0f);
    ros::Duration(2).sleep();
    //    arm_controller_->moveArmJoint(nonGraspingHand, 3, -1*tempOffset);
    //    ros::Duration(1).sleep();
    gripper_controller_->controlGripper(graspingHand, GRIPPER_STATE::TIGHT_HOLD);

}

void task2Utils::raisePanel(const armSide graspingHand)
{
    const std::vector<float> *graspingHandPoseUp;
    if(graspingHand == armSide::LEFT){
        //        graspingHandPoseUp = &leftNearChestPalmDown_;
        graspingHandPoseUp = &leftPanelPlacementUpPose1_;
    }
    else
    {
        //        graspingHandPoseUp = &rightNearChestPalmDown_;
        graspingHandPoseUp = &rightPanelPlacementUpPose1_;
    }

    std::vector< std::vector<float> > armData;
    armData.clear();
    armData.push_back(*graspingHandPoseUp);
    arm_controller_->moveArmJoints(graspingHand, armData, 2.0f);
    ros::Duration(2).sleep();
}

void task2Utils::reOrientTowardsGoal(geometry_msgs::Point goal_point, float offset){



    size_t nSteps;
    armSide startStep;
    std::vector<float> y_offset;
    std::vector<float> x_offset;

    current_state_->transformPoint(goal_point,goal_point,VAL_COMMON_NAMES::WORLD_TF,
                                   rd_->getPelvisFrame());

    double error = goal_point.y+offset;
    double abserror = std::fabs(error);
    ROS_INFO_STREAM("Error is:" << error << "Absolute value for error is:" << abserror);
    if (abserror < 0.1 || abserror > 0.49 ){
        ROS_INFO("reOrientTowardsPanel: Not reorienting, the offset is too less or beyond control");
    }
    else
    {
        pelvis_controller_->controlPelvisHeight(1.0);
        ros::Duration(1.5).sleep();

        nSteps = int(((abserror/0.1)+0.5));
        ROS_INFO_STREAM("No of steps to walk is:" << nSteps);

        if (error > 0){
            startStep = LEFT;
            for(size_t i = 0; i < nSteps; ++i){
                y_offset.push_back(0.1);
                y_offset.push_back(0.1);
            }
        }
        else {
            startStep = RIGHT;
            for(size_t i = 0; i < nSteps; ++i){
                y_offset.push_back(-0.1);
                y_offset.push_back(-0.1);
            }
        }

        x_offset.resize(y_offset.size());
        ROS_INFO("reOrientTowardsPanel: Walking %d steps",int(nSteps));

        walk_->walkLocalPreComputedSteps(x_offset,y_offset,startStep);
        ros::Duration(4.0).sleep();

        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(1.5).sleep();
    }


}

// this function is not being used anywhere. maybe it's not needed. ///@todo delete
void task2Utils::reOrientTowardsCable(geometry_msgs::Pose cablePose, geometry_msgs::Pose panelPose){

    geometry_msgs::Pose poseInPelvisFrame;
    current_state_->transformPose(cablePose, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());

    float yaw = tf::getYaw(poseInPelvisFrame.orientation);

    ROS_INFO("Yaw Value : %f",yaw);

    // if the vector is pointing outwards, reorient it
    if (yaw > M_PI_2 || yaw < -M_PI_2){
        geometry_msgs::Pose tempYaw(cablePose);
        SolarPanelDetect::invertYaw(tempYaw);

        current_state_->transformPose(tempYaw, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
        yaw = tf::getYaw(poseInPelvisFrame.orientation);
    }

    ROS_INFO("reOrient towards cable: The current Yaw in pelvis frame is: %d", yaw);

    //    geometry_msgs::Pose cableOrientationPose;
    //    // Choose the cableOrientation pose based on the condition
    //    //side = yaw < 0 ? armSide::LEFT : armSide::RIGHT;

    //    geometry_msgs::Pose robotPose;
    //    current_state_->getCurrentPose(VAL_COMMON_NAMES::WORLD_TF,robotPose);

    //    geometry_msgs::Pose2D cableOrientationPose2D;
    //    cableOrientationPose2D.x = cableOrientationPose.position.x;
    //    cableOrientationPose2D.y = cableOrientationPose.position.y;
    //    cableOrientationPose2D.theta = tf::getYaw(robotPose.orientation);

    //    //Convert cableo... pose to world and call the walker
    //    walk_->walkToGoal(cableOrientationPose2D);

}

int task2Utils::getCurrentCheckpoint() const{
    return current_checkpoint_;
}

bool task2Utils::isCableOnTable(geometry_msgs::Pose &cable_pose)
{
    // this function checks the z coordinate of the cable location and verifies if this matches the height of the table with some tolerance

    float tolerance =0.1; // experimental value
    return ((cable_pose.position.z < table_height_ + tolerance) && (cable_pose.position.z > table_height_ - tolerance));

}



bool task2Utils::isCableInHand(armSide side)
{
    // this function rotates the hand slighly to detect the cable and brings it back to same position
    std::string arm = side == LEFT ? "left_arm" : "right_arm";
    std::vector<float> jointPositions;
    current_state_->getJointPositions(arm,jointPositions);

    std::vector< std::vector<float> > armData;
    armData.push_back(righCableInHandSeed_);
    arm_controller_->moveArmJoints(side, armData,2.0f);
    ros::Duration(3).sleep();

    ///@todo detection part goes here

    //    armData.clear();
    //    armData.push_back(jointPositions);
    //    arm_controller_->moveArmJoints(side, armData,2.0f);
    //    ros::Duration(0.2).sleep();

    return true;

}

bool task2Utils::isCableTouchingSocket()
{
    if(current_checkpoint_ == 5 && taskMsg.checkpoint_durations.size() > 3 ){
        taskLogPub(TEXT_GREEN + "Cable is touching socket" + TEXT_NC);
        return true;
    }

    return false;
}

geometry_msgs::Pose task2Utils::grasping_hand(armSide &side, geometry_msgs::Pose handle_pose)
{
    geometry_msgs::Pose poseInPelvisFrame;
    current_state_->transformPose(handle_pose, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
    float yaw = tf::getYaw(poseInPelvisFrame.orientation);

    if (yaw > M_PI_2 || yaw < -M_PI_2){
        tfScalar r, p, y;
        tf::Quaternion q;
        tf::quaternionMsgToTF(handle_pose.orientation, q);
        tf::Matrix3x3 rot(q);
        rot.getRPY(r, p, y);
        y = y-M_PI;
        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);
        handle_pose.orientation = quaternion;
        current_state_->transformPose(handle_pose, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
        yaw = tf::getYaw(poseInPelvisFrame.orientation);
    }

    side = yaw < 0 ? armSide::LEFT : armSide::RIGHT;
    return handle_pose;
}

bool task2Utils::isRotationReq(armSide side, geometry_msgs::Point handle_coordinates,geometry_msgs::Point button_coordinates)
{
    bool is_rotation_required_;
    if(button_coordinates.x == 0 && button_coordinates.y == 0){
        // button cannot be seen it is assumed to be on the other side
        is_rotation_required_ = side == armSide::LEFT ? false : true;
    }
    else{
        current_state_->transformPoint(handle_coordinates, handle_coordinates, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());
        current_state_->transformPoint(button_coordinates, button_coordinates, VAL_COMMON_NAMES::WORLD_TF, rd_->getPelvisFrame());

        if( button_coordinates.x > handle_coordinates.x) {
            is_rotation_required_ = side == armSide::LEFT ? false : true;
        }
        else if (handle_coordinates.x == button_coordinates.x &&  button_coordinates.y > handle_coordinates.y ){
            is_rotation_required_ = false;
        }
        else if (handle_coordinates.x == button_coordinates.x &&  button_coordinates.y < handle_coordinates.y ){
            is_rotation_required_ = true;
        }
        else{
            // x is smaller
            is_rotation_required_ = side == armSide::LEFT ? true : false;
        }
    }
    return is_rotation_required_;
}

bool task2Utils::checkpoint_init()
{
    ROS_INFO("[SKIP] Resetting point cloud and map for skipped checkpoint");
    taskLogPub("[SKIP] Resetting point cloud and map for skipped checkpoint");

    ros::Duration(3).sleep(); // 3 seconds sleep to get the most recent-new point cloud
    clearPointCloud();
    ros::Duration(1).sleep();
    clearMap();
    ros::Duration(1).sleep();

    walk_->walkLocalPreComputedSteps({-0.2,-0.2,-0.4,-0.4},{0.0,0.0,0.0,0.0},RIGHT);
    ros::Duration(3).sleep();
    head_controller_->moveHead(0,40,0);
    ros::Duration(10).sleep();
    head_controller_->moveHead(0,0,40);
    ros::Duration(5).sleep();
    head_controller_->moveHead(0,0,-40);
    ros::Duration(5).sleep();
    head_controller_->moveHead(0,0,0);
    ROS_INFO("[SKIP] Setting pelvis height");
    taskLogPub("[SKIP] Setting pelvis height");
    pelvis_controller_->controlPelvisHeight(0.9);
    ros::Duration(1).sleep();;

}

bool task2Utils::shakeTest(const armSide graspingHand)
{
    return true;
    ROS_INFO("task2Utils::shakeTest : Closing, opening and reclosing grippers to see if the panel falls off");
    //close
    //open
    //close

    // If the bag is on the tip of the finger it would slip in or fall out giving a better grasp
    gripper_controller_->closeGripper(graspingHand);
    ros::Duration(0.2).sleep();
    gripper_controller_->openGripper(graspingHand);
    ros::Duration(0.2).sleep();
    gripper_controller_->closeGripper(graspingHand);
    ros::Duration(0.2).sleep();

    return true;

}
/**
 * @brief task2Utils::pushDeployedPanel walks a fixed step back, moves both arms in position and moves the same fixed distance forward to push the deployed panel.
 * After pushing the panel, the socket is in a more convenient position to attempt cable plug-in
 * @return success
 */
bool task2Utils::pushDeployedPanel()
{
    std::vector<float> x_offset={-0.2, -0.4, -0.4};
    std::vector<float> y_offset={0.0, 0.0, 0.0};
    walk_->walkLocalPreComputedSteps(x_offset,y_offset,LEFT);
    ros::Duration(4).sleep();

    x_offset={0.2, 0.4, 0.4};
    y_offset={0.0, 0.0, 0.0};

    std::vector< std::vector<float> > armData;
    std::vector<float> leftHandData = {-0.23, -1.24, 0.07, -1.25, 1.23, 0.0, 0.0};
    std::vector<float> rightHandData = {-0.42, 1.38, 0.60, 1.15,  1.28, 0.0, 0.0};

    armData.clear();
    armData.push_back(leftHandData);
    arm_controller_->moveArmJoints(armSide::LEFT, armData, 0.2f);
    armData.clear();
    armData.push_back(rightHandData);
    arm_controller_->moveArmJoints(armSide::RIGHT, armData, 2.0f);

    ros::Duration(2).sleep();
    walk_->walkLocalPreComputedSteps(x_offset,y_offset,LEFT);
    ros::Duration(4).sleep();

    return true;

}


void task2Utils::taskStatusCB(const srcsim::Task &msg)
{
    taskMsg = msg;
    if(current_checkpoint_ == 5 && taskMsg.checkpoint_durations.size() > 3 ){
        taskLogPub(TEXT_GREEN + "Cable is touching socket" + TEXT_NC);
    }

    if (msg.current_checkpoint != current_checkpoint_){
        current_checkpoint_ = msg.current_checkpoint;

        std::ofstream outfile(logFile, std::ofstream::app);
        std::stringstream data;

        data << boost::posix_time::to_simple_string(timeNow) << "," << msg.task << ","
             << msg.current_checkpoint << "," << msg.elapsed_time << std::endl;
        ROS_INFO("task2Utils::taskStatusCB : Current checkpoint : %d", current_checkpoint_);

        outfile << data.str();
        outfile.close();

        taskLogPub(TEXT_GREEN + "Current Checkpoint : "+ std::to_string(current_checkpoint_) + TEXT_NC);
    }

}

void task2Utils::mapUpdateCB(const nav_msgs::OccupancyGrid &msg)
{
    mtx_.lock();
    map_ = msg;
    mtx_.unlock();
}

void task2Utils::clearPointCloud() {
    std_msgs::Empty msg;
    reset_pointcloud_pub.publish(msg);
    ros::Duration(0.3).sleep();
}

void task2Utils::clearCurrentPoseMap() {
    std_msgs::Empty msg;
    clear_pose_map.publish(msg);
    ros::Duration(0.3).sleep();
}

void task2Utils::clearMap() {
    std_msgs::Empty msg;
    reset_map_pub.publish(msg);
    ros::Duration(0.3).sleep();
}

void task2Utils::clearBoxPointCloud(CLEAR_BOX_CLOUD state) {
    std_msgs::Int8 msg;
    if(state == CLEAR_BOX_CLOUD::WAIST_UP)
    {
        msg.data =1;
    }
    else if(state == CLEAR_BOX_CLOUD::LARGE_BOX)
    {
        msg.data =2;
    }
    else msg.data =5;


    clearbox_pointcloud_pub.publish(msg);
    ros::Duration(0.3).sleep();
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

void task2Utils::taskLogPub(std::string data){
    static std::string prev_msg = "";
    if(prev_msg != data){
        std_msgs::String ms;
        ms.data = data;
        task2_log_pub_.publish(ms);
        prev_msg = data;
    }
}

bool task2Utils::planWholeBodyMotion(armSide side, std::vector<geometry_msgs::Pose> waypoints)
{
    moveit_msgs::RobotTrajectory traj;
    if(side ==armSide::RIGHT)
    {
        if (right_arm_planner_->getTrajFromCartPoints(waypoints, traj, false)> 0.98){
            ROS_INFO("right arm whole body msg executed");
            wholebody_controller_->compileMsg(side, traj.joint_trajectory);
        }
    }
    else
    {
        if (left_arm_planner_->getTrajFromCartPoints(waypoints, traj, false)> 0.98){
            ROS_INFO("left arm whole body msg executed");
            wholebody_controller_->compileMsg(side, traj.joint_trajectory);
        }

    }
}
