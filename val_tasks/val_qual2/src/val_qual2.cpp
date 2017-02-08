#include <ros/ros.h>
#include <val_control/val_arm_navigation.h>
#include <val_control/val_pelvis_navigation.h>
#include <val_footstep/ValkyrieWalker.h>
#include <val_control/val_chest_navigation.h>
#include <ihmc_msgs/ChestTrajectoryRosMessage.h>
#include <ihmc_msgs/FootTrajectoryRosMessage.h>
#include <tf2/utils.h>

enum sm {
    PREPARE_START = 0,
    WALK_TO_DOOR
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Qual2");
    ros::NodeHandle nh;


    armTrajectory armtraj(nh);
    pelvisTrajectory pelvisTraj(nh);

    float transferTime=0.5, swingTime=0.5, swingHeight=0.15, pelvisHeight=1.06;
    if ( argc != 5 ) // 5 arguments required
    {
        std::cout<<"The values are not set, using the default values for run..!!!!!"<< std::endl;
        std::cout<<"transferTime: "<<transferTime<<"\n"<<"swingTime: "<<swingTime<<"\n"<<"swingHeight: "<<swingHeight<<"\n"<<"pelvisHeight: "<<pelvisHeight<<"\n";
    }
    else
    {
        transferTime = std::atof(argv[1]);
        swingTime = std::atof(argv[2]);
        swingHeight = std::atof(argv[3]);
        pelvisHeight = std::atof(argv[4]);

        std::cout<<"Using the folloing paramteres for walking:"<< std::endl;
        std::cout<<"transferTime: "<<transferTime<<"\n"<<"swingTime: "<<swingTime<<"\n"<<"swingHeight: "<<swingHeight<<"\n"<<"pelvisHeight: "<<pelvisHeight<<"\n";
    }

    // optimum values for walking
    //ValkyrieWalker walk(nh, 0.34, 0.44);

    ValkyrieWalker walk(nh, transferTime, swingTime);
    walk.setSwing_height(swingHeight);

    sm state = PREPARE_START;
    // The code structure is so as to get the fastest and optimum execution
    // sm is completely dismembered, process cannot slow us down!
    switch (state)
    {
    case PREPARE_START:
    {
        pelvisTraj.controlPelvisHeight(pelvisHeight);

        ros::Duration(0.5).sleep();
        std::vector<armTrajectory::armJointData> arm_data;
        armTrajectory::armJointData arm_data_seq1, arm_data_seq2, arm_data_seq3,
                arm_data_seq4, arm_data_seq5, arm_data_seq6, arm_data_seq7,
                arm_data_seq8, arm_data_seq9, arm_data_seq10,
                arm_data_seq11, arm_data_seq12, arm_data_seq13, arm_data_seq14;

        arm_data_seq1.side = RIGHT; // retract to actual
        arm_data_seq1.arm_pose = {-0.2, 1.2, 0.65, 1.5, 0.0, 0.0, 0.0};
        arm_data_seq1.time = 3;

        arm_data_seq2.side = LEFT;
        arm_data_seq2.arm_pose = {-0.2, -1.2, 0.65, -1.5, 0.0, 0.0, 0.0};
        arm_data_seq2.time = 3;

        arm_data_seq3.side = RIGHT; // stable walk
        arm_data_seq3.arm_pose = {0.0, 0.25, 0.2, 0.9, 0.0, 0.0, 0.0};
        arm_data_seq3.time = 4;

        arm_data_seq4.side = LEFT;
        arm_data_seq4.arm_pose = {0.0, -0.25, 0.2, -0.9, 0.0, 0.0, 0.0};
        arm_data_seq4.time = 4;

        arm_data_seq5.side = RIGHT; // stable walk
        arm_data_seq5.arm_pose = {0.0, 0.25, 0.2, 0.9, 0.0, 0.0, 0.0};
        arm_data_seq5.time = 6.5;

        arm_data_seq6.side = LEFT;
        arm_data_seq6.arm_pose = {0.0, -0.25, 0.2, -0.9, 0.0, 0.0, 0.0};
        arm_data_seq6.time = 6.5;

        arm_data_seq7.side = RIGHT; // button press prepare
        arm_data_seq7.arm_pose = {1.5, -0.4, -0.65, 0.8, 0.0, 0.0, 0.0};
        arm_data_seq7.time = 7.6;

        arm_data_seq8.side = LEFT;
        arm_data_seq8.arm_pose = {1.5, -0.4, -0.65, -0.7, 0.0, 0.0, 0.0};
        arm_data_seq8.time = 7.8;

        arm_data_seq9.side = RIGHT; // zero position
        arm_data_seq9.arm_pose = {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0};
        arm_data_seq9.time = 7.8;

        arm_data_seq10.side = LEFT;
        arm_data_seq10.arm_pose = {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0};
        arm_data_seq10.time = 7.8;

        arm_data_seq11.side = RIGHT; // intermideate point r
        arm_data_seq11.arm_pose = {1.5, -0.0, -0.1, 1.7, 0.0, 0.0, 0.0};
        arm_data_seq11.time = 7.9;

        arm_data_seq12.side = RIGHT; // retract to actual
        arm_data_seq12.arm_pose = {-0.2, 1.2, 0.65, 1.5, 0.0, 0.0, 0.0};
        arm_data_seq12.time = 7.9;

        arm_data_seq13.side = RIGHT; // retract to actual
        arm_data_seq13.arm_pose = {-0.2, 1.2, 0.65, 1.5, 0.0, 0.0, 0.0};
        arm_data_seq13.time = 8.1;

        arm_data_seq14.side = LEFT;
        arm_data_seq14.arm_pose = {-0.2, -1.2, 0.65, -1.5, 0.0, 0.0, 0.0};
        arm_data_seq14.time = 8.00;

        arm_data.push_back(arm_data_seq1);
        arm_data.push_back(arm_data_seq2);
        arm_data.push_back(arm_data_seq3);
        arm_data.push_back(arm_data_seq4);
        arm_data.push_back(arm_data_seq5);
        arm_data.push_back(arm_data_seq6);
        arm_data.push_back(arm_data_seq7);
        arm_data.push_back(arm_data_seq8);
        arm_data.push_back(arm_data_seq9);
        arm_data.push_back(arm_data_seq10);
        arm_data.push_back(arm_data_seq11);
        arm_data.push_back(arm_data_seq12);
        arm_data.push_back(arm_data_seq13);
        arm_data.push_back(arm_data_seq14);

        ///get the armtrajectory message from the arm_trajectory*.cpp
        /// and modify the hardcoded part from buttonpress function and publish the message, foverload struct
        armtraj.moveArmJoints(arm_data);
        state = WALK_TO_DOOR;

        // fall through the case, so we save one iteration (0.1 sec)
        //break;
    }
    case WALK_TO_DOOR:
    {
        walk.setWalkParms(transferTime, swingTime, 0);
        // each step is 0.51m offset except for the first and seventh. Why? Guy with Mad Skills told so
//        std::vector<float> x_offset = {0.35,0.86,1.37,1.88,2.39,2.9,3.41,3.818,4.328,4.838,4.838};
        std::vector<float> x_offset = {0.35, 0.85, 1.35, 1.85, 2.35, 2.85, 3.35, 3.7, 4.2, 4.7, 5.2};
        std::vector<float> y_offset = {0,0,0,0,0,0,0,0,0,0,0};
        walk.walkPreComputedSteps(x_offset, y_offset, RIGHT);
        break;
    }

    default:
        break;
    }

    ros::spinOnce();
    return 0;
}
