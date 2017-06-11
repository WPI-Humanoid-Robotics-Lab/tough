#include "val_task3/val_stairclimb.h"
#include "val_common/val_common_defines.h"
#include "val_footstep/ValkyrieWalker.h"


StairClimb::StairClimb(ros::NodeHandle n)
{
    walker_ = new ValkyrieWalker(n, 0.7, 0.7, 0, 0.18);
    robot_state_=RobotStateInformer::getRobotStateInformer(n);
    chest_controller_ = new chestTrajectory(n);

}

bool StairClimb::walkToSetPosition(geometry_msgs::Pose2D goal)
{
    walker_->walkToGoal(goal);

}

bool StairClimb::takeStep(armSide side,geometry_msgs::Pose nextStair)
{
    /*
     * Stair case climbing involves six stages
     * Stage 1: Lift Leg vertically high to 1.5 times the step height
     * Stage 2: Place Leg on the Step
     * Stage 3: Bend the chest forward and move the hands up
     * Stage 4: Lift the swing leg in a way to curl behind the body
     * Stage 5: Place swing foot on the stair
     * Stage 6: Orient the chest back to zero position
    */

    float raise_factor=1.5;
    float lower_factor=-0.5;
    float pitch_angle=20;
    float curl_radius=0.2;

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    walker_->getCurrentStep(side, *current);
    geometry_msgs::Pose currentPelvis;
    currentPelvis.position.x=current->location.x;
    currentPelvis.position.y=current->location.y;
    currentPelvis.position.z=current->location.z;
    currentPelvis.orientation=current->orientation;

    robot_state_->transformPose(nextStair,nextStair,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
    robot_state_->transformPose(currentPelvis,currentPelvis,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);

    float step_length = fabs(currentPelvis.position.x-nextStair.position.x);
    float step_height = fabs(currentPelvis.position.z-nextStair.position.z);


    // second leg movement
    walker_->curlLeg(side,curl_radius);
    ros::Duration(1).sleep();
    walker_->placeLeg(side,raise_factor*step_height);
    ros::Duration(1).sleep();
    walker_->nudgeFoot(side,step_length);
    ros::Duration(1).sleep();
    walker_->raiseLeg(side,lower_factor*step_height,0.0);
    ros::Duration(1).sleep();
    walker_->load_eff(side,EE_LOADING::LOAD);
    ros::Duration(1).sleep();
    chest_controller_->controlChest(0,pitch_angle,0);
    ros::Duration(1).sleep();
}


bool StairClimb::takeFistStep(armSide side,geometry_msgs::Pose firstStair)
{
    float raise_factor=1.5;
    float lower_factor=-0.5;
    float pitch_angle=20;

    //     find current position and first stair position

    ihmc_msgs::FootstepDataRosMessage::Ptr current(new ihmc_msgs::FootstepDataRosMessage());
    walker_->getCurrentStep(side, *current);
    geometry_msgs::Pose currentPelvis;
    currentPelvis.position.x=current->location.x;
    currentPelvis.position.y=current->location.y;
    currentPelvis.position.z=current->location.z;
    currentPelvis.orientation=current->orientation;

    robot_state_->transformPose(firstStair,firstStair,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);
    robot_state_->transformPose(currentPelvis,currentPelvis,VAL_COMMON_NAMES::WORLD_TF,VAL_COMMON_NAMES::PELVIS_TF);

    float step_length = fabs(currentPelvis.position.x-firstStair.position.x);
    float step_height = fabs(currentPelvis.position.z-firstStair.position.z);

    // fist leg movement
    walker_->raiseLeg(side,raise_factor*step_height,0.0);
    ros::Duration(1).sleep();
    walker_->nudgeFoot(side,step_length);
    ros::Duration(1).sleep();
    walker_->raiseLeg(side,lower_factor*step_height,0.0);
    ros::Duration(1).sleep();
    walker_->load_eff(side,EE_LOADING::LOAD);
    ros::Duration(1).sleep();
    chest_controller_->controlChest(0,pitch_angle,0);
    ros::Duration(1).sleep();
}

int main(int argc, char** argv){

    ros::init(argc, argv, "stair_climb",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    ValkyrieWalker walk(nh, 1.0,1.0,0);
    StairClimb stair(nh);

    stair_detector_2 detector(nh);
    std::vector<geometry_msgs::Pose> detections;

    while(!detector.getDetections(detections)) {
        ROS_INFO_STREAM_THROTTLE(5, "Stairs detections: " << detections.size());
        ros::spinOnce();
    }


    std::vector<float> step_length,step_height;

    geometry_msgs::Pose2D goal;
    goal.x=detections[0].position.x;
    goal.y=detections[0].position.y;
    goal.theta=tf::getYaw(detections[0].orientation);
    stair.walkToSetPosition(goal);
    ros::Duration(1).sleep();

    for (int i = 0; i < detections.size(); ++i) {
        std::cout<<"x: "<<detections[i].position.x<<" y: "<<detections[i].position.y<<" z: "<<detections[i].position.z<<"\n";
    }


    for (int i = 1; i < 10; ++i) {
        step_height.push_back(detections[i].position.z-detections[i-1].position.z);
        step_length.push_back(detections[i].position.x-detections[i-1].position.x);  /// @todo change to pelvis and then compute
        //        std::cout<<"step height at "<<i<<"is"<<(detections[i].position.z-detections[i-1].position.z)<<"\n";
    }

    //    step_length.assign(10,0.24);

    armSide side;
    stair.takeFistStep(RIGHT,detections[0]);
    for (int i = 1; i < detections.size(); ++i) {
        side = i%2==1 ? armSide::LEFT : armSide::RIGHT;
        stair.takeStep(side,detections[i]);

    };
    ros::Duration(2).sleep();
    return 0;
}
