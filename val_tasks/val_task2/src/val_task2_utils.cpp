#include <val_task2/val_task2_utils.h>


task2Utils::task2Utils(ros::NodeHandle nh):
    nh_(nh)
{
    // controllers
    chest_controller_    = new chestTrajectory(nh_);
    pelvis_controller_   = new pelvisTrajectory(nh_);
    head_controller_     = new HeadTrajectory(nh_);
    gripper_controller_  = new gripperControl(nh_);
    arm_controller_      = new armTrajectory(nh_);
}

task2Utils::~task2Utils()
{

}

void task2Utils::afterPanelGraspPose(const armSide side)
{
    // reorient the chest
    chest_controller_->controlChest(0,0,0);
    ros::Duration(2).sleep();

    // walk a step back
    std::vector<float> x_offset={-0.2,-0.2};
    std::vector<float> y_offset={0.0,0.0};
    walk->walkLocalPreComputedSteps(x_offset,y_offset,RIGHT);

    const std::vector<float> *seed1,*seed2;
    if(side == armSide::LEFT){
        seed1 = &leftShoulderSeedPanelGraspStatic_;
        seed2 = &leftShoulderSeedPanelGraspWalk_;
    }
    else
    {
        seed1 = &rightShoulderSeedPanelGraspStatic_;
        seed2 = &rightShoulderSeedPanelGraspWalk_;
    }

    std::vector< std::vector<float> > armData;
    armData.push_back(*seed1);

    arm_controller_->moveArmJoints(side, armData, 2.0f);
    ros::Duration(2).sleep();

    armData.clear();
    armData.push_back(*seed2);

    arm_controller_->moveArmJoints(side, armData, 2.0f);
    ros::Duration(2).sleep();
}

