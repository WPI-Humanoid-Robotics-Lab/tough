#include <src_task3/val_task3_utils.h>
#include <math.h>

task3Utils::task3Utils(ros::NodeHandle nh): nh_(nh),arm_controller_(nh_)
{
    current_state_       = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
    pelvis_controller_   = new pelvisTrajectory(nh_);
    walk_                = new RobotWalker(nh_, 0.7, 0.7, 0, 0.18);

    visited_map_sub_  = nh_.subscribe("/visited_map",10, &task3Utils::visited_map_cb, this);
    task_status_sub_  = nh_.subscribe("/srcsim/finals/task", 10, &task3Utils::taskStatusCB, this);
    task3_log_pub_    = nh_.advertise<std_msgs::String>("/field/log",10);
    is_climbstairs_finished_ = false;
    current_checkpoint_ = 0;
}

task3Utils::~task3Utils(){
    delete pelvis_controller_   ;
    delete walk_                ;

    // shut down subscribers
    visited_map_sub_.shutdown();
    task_status_sub_.shutdown();
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

     arm_controller_.moveArmJoints(RIGHT,armData,0.5f);
     ros::Duration(2.0).sleep();

     armData.clear();
     armData.push_back(LEFT_ARM_DOOR_OPEN);
     arm_controller_.moveArmJoints(LEFT,armData,0.5f);
     ros::Duration(2.0).sleep();
}

void task3Utils::doorWalkwayPose()
{
    std::vector< std::vector<float> > armData;
    armData.push_back(RIGHT_ARM_DOORWAY_WALK);

     arm_controller_.moveArmJoints(RIGHT,armData,1.0f);
     ros::Duration(2.0).sleep();

     armData.clear();
     armData.push_back(LEFT_ARM_DOORWAY_WALK);
     arm_controller_.moveArmJoints(LEFT,armData,1.0f);
     ros::Duration(2.0).sleep();
}


void task3Utils::visited_map_cb(const nav_msgs::OccupancyGrid::Ptr msg)
{
    visited_map_ = *msg;
}

void task3Utils::taskStatusCB(const srcsim::Task &msg)
{
    task_msg_ = msg;

    // if climb stairs task is fnished
    if (task_msg_.task == 3 &&
        task_msg_.current_checkpoint == 1 &&
        task_msg_.finished == true)
    {
        // scoped mutex
        std::lock_guard<std::mutex> lock(climstairs_flag_mtx_);
        is_climbstairs_finished_ = true;
    }

    if (current_checkpoint_ != msg.current_checkpoint){
        current_checkpoint_ = msg.current_checkpoint;
        task3LogPub(TEXT_GREEN + "Current Checkpoint : "+ std::to_string(current_checkpoint_) + TEXT_NC);
    }
}

void task3Utils::blindNavigation(geometry_msgs::Pose2D & goal){
    // get current state
    RobotStateInformer *current_state = RobotStateInformer::getRobotStateInformer(nh_);
    geometry_msgs::Pose poseInPelvisFrame, tempPose;
    current_state->getCurrentPose(rd_->getPelvisFrame(),poseInPelvisFrame);

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

// Copied from task2utils
geometry_msgs::Pose task3Utils::grasping_hand(RobotSide &side, geometry_msgs::Pose handle_pose) {
    geometry_msgs::Pose poseInPelvisFrame;
    current_state_->transformPose(handle_pose, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF,
                                  rd_->getPelvisFrame());
    float yaw = tf::getYaw(poseInPelvisFrame.orientation);

    if (yaw > M_PI_2 || yaw < -M_PI_2) {
        tfScalar r, p, y;
        tf::Quaternion q;
        tf::quaternionMsgToTF(handle_pose.orientation, q);
        tf::Matrix3x3 rot(q);
        rot.getRPY(r, p, y);
        y = y - M_PI;
        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(r, p, y);
        handle_pose.orientation = quaternion;
        current_state_->transformPose(handle_pose, poseInPelvisFrame, VAL_COMMON_NAMES::WORLD_TF,
                                      rd_->getPelvisFrame());
        yaw = tf::getYaw(poseInPelvisFrame.orientation);
    }

    side = yaw < 0 ? RobotSide::LEFT : RobotSide::RIGHT;
    return handle_pose;
}

bool task3Utils::isClimbstairsFinished() const
{
    return is_climbstairs_finished_;
}

void task3Utils::resetClimbstairsFlag(void)
{
    // scoped mutex
    std::lock_guard<std::mutex> lock(climstairs_flag_mtx_);
    is_climbstairs_finished_ = false;
}

void task3Utils::task3LogPub(std::string data){

    std_msgs::String ms;
    ms.data = data;
    task3_log_pub_.publish(ms);
}

// Copied and modified from task2utils
void task3Utils::walkSideways(float error) {
    std::size_t nSteps;
    RobotSide startStep;
    std::vector<float> y_offset;
    std::vector<float> x_offset;

    double abserror = std::fabs(error);
    ROS_INFO_STREAM("Error is:" << error << "Absolute value for error is:" << abserror);
    if (abserror < 0.05){
        ROS_INFO("reOrientTowardsPanel: Not reorienting, the offset is less than 0.05");
    }
    else
    {
        pelvis_controller_->controlPelvisHeight(1.0);
        ros::Duration(1.5).sleep();

        nSteps = static_cast<std::size_t>(((abserror/0.1)+0.5));
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
                y_offset.push_back(-0.1f);
                y_offset.push_back(-0.1f);
            }
        }

        x_offset.resize(y_offset.size(), 0);
        ROS_INFO("reOrientTowardsPanel: Walking %d steps",int(nSteps));

        walk_->walkLocalPreComputedSteps(x_offset, y_offset, startStep);
        ros::Duration(4.0).sleep();

        pelvis_controller_->controlPelvisHeight(0.9);
        ros::Duration(1.5).sleep();
    }

}
