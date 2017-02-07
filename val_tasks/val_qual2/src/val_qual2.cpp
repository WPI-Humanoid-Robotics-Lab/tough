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
    WALK_TO_DOOR,
    EXIT
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Qual2");
    ros::NodeHandle nh;


    armTrajectory armtraj(nh);
    pelvisTrajectory pelvisTraj(nh);

    float transferTime=0.35, swingTime=0.45, swingHeight=0.13, pelvisHeight=1.07;
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
    // The code structure is so to get the fastest and optimum execution
    switch (state)
    {
    case PREPARE_START:
    {
        pelvisTraj.controlPelvisHeight(pelvisHeight);

       // walk.WalkNStepsForward(11,0.35,0,false,RIGHT);
        ros::Duration(0.5).sleep();
        std::vector<armTrajectory::moveArmData> arm_data;
        armTrajectory::moveArmData arm_data_seq1, arm_data_seq2, arm_data_seq3,
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
        arm_data_seq5.time = 5.3;

        arm_data_seq6.side = LEFT;
        arm_data_seq6.arm_pose = {0.0, -0.25, 0.2, -0.9, 0.0, 0.0, 0.0};
        arm_data_seq6.time = 5.3;

        arm_data_seq7.side = RIGHT; // button press prepare
        arm_data_seq7.arm_pose = {1.5, -0.4, -0.65, 0.7, 0.0, 0.0, 0.0};
        arm_data_seq7.time = 5.75;

        arm_data_seq8.side = LEFT;
        arm_data_seq8.arm_pose = {1.5, -0.4, -0.65, -0.7, 0.0, 0.0, 0.0};
        arm_data_seq8.time = 5.75;

        arm_data_seq9.side = RIGHT; // zero position
        arm_data_seq9.arm_pose = {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0};
        arm_data_seq9.time = 5.75;

        arm_data_seq10.side = LEFT;
        arm_data_seq10.arm_pose = {0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0};
        arm_data_seq10.time = 5.75;

        arm_data_seq11.side = RIGHT; // arm back r
        arm_data_seq11.arm_pose = {1.3, 1.2, 2.0, 0.0, 0.0, 0.0, 0.0};
        arm_data_seq11.time = 7.0;

        arm_data_seq12.side = RIGHT; // intermideate point r
        arm_data_seq12.arm_pose = {1.5, -0.0, -0.1, 1.7, 0.0, 0.0, 0.0};
        arm_data_seq12.time = 7.5;

        arm_data_seq13.side = RIGHT; // retract to actual
        arm_data_seq13.arm_pose = {-0.2, 1.2, 0.65, 1.5, 0.0, 0.0, 0.0};
        arm_data_seq13.time = 8.0;

        arm_data_seq14.side = LEFT;
        arm_data_seq14.arm_pose = {-0.2, -1.2, 0.65, -1.5, 0.0, 0.0, 0.0};
        arm_data_seq14.time = 6.75;

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
        armtraj.moveArm(arm_data);
        state = WALK_TO_DOOR;

        // fall through the case, so we save one iteration (0.1 sec)
        //break;
    }
    case WALK_TO_DOOR:
    {
        walk.setWalkParms(transferTime, swingTime, 0);
        ///get the footsteplist message and modify the footsteps based on
        /// what is hardcoded in ValkyrieWalker.cpp and publish it again
        std::vector<float> x_offset = {0.35,0.86,1.37,1.88,2.39,2.9,3.41,3.818,4.328,4.838,4.838};
        std::vector<float> y_offset = {0,0,0,0,0,0,0,0,0,0,0};
        int no_steps = 12;
//        walk.WalkPreComputedSteps(no_steps, x_offset, y_offset, false, RIGHT);
        //walk.WalkNStepsForward(12,0.51,0, false, RIGHT);
        break;
    }

    default:
EXIT:
        break;
    }

    ros::spinOnce();
    return 0;
}


bool OrientChest(float roll, float pitch, float yaw, ros::Publisher pub){
    ihmc_msgs::ChestTrajectoryRosMessage msg;
    std::vector<ihmc_msgs::SO3TrajectoryPointRosMessage> trajPointsVec;
    ihmc_msgs::SO3TrajectoryPointRosMessage trajPoint1, trajPoint2;
    tf::Quaternion angles;
    angles.setRPY((tfScalar)roll*3.1427/180, (tfScalar)pitch*3.1427/180, (tfScalar)yaw*3.1427/180);

    geometry_msgs::Quaternion angles1;
    angles1.x = angles.getX();
    angles1.y = angles.getY();
    angles1.z = angles.getZ();
    angles1.w = angles.getW();

    trajPoint1.orientation = angles1;
    trajPoint1.time = 3.0;
    trajPointsVec.push_back(trajPoint1);

    geometry_msgs::Quaternion angles2;
    angles2.x = 0.0;
    angles2.y = 0.0;
    angles2.z = 0.0;
    angles2.w = 1.0;

    trajPoint2.orientation = angles2;
    trajPoint2.time = 0.5;
    trajPointsVec.push_back(trajPoint2);


    msg.execution_mode = 0;
    msg.unique_id = 13;
    msg.taskspace_trajectory_points = trajPointsVec;

    pub.publish(msg);
    //ROS_INFO("Published chest messages");

}

//void footStepTraj(ros::Publisher pub)
//{
//    ihmc_msgs::FootTrajectoryRosMessage msgR;
//    ihmc_msgs::FootTrajectoryRosMessage msgL;
//    ihmc_msgs::SE3TrajectoryPointRosMessage trajPointR, trajPointL;

//    msgR.execution_mode=0;
//    msgR.robot_side = RIGHT;
//    msgR.previous_message_id = 0;
//    msgR.taskspace_trajectory_points.clear();
//    msgR.unique_id = 40;

//    trajPointR.unique_id = 41; //(int)ros::Time::now();
//    trajPointR.time = 0.0; // reach instantenously
//    trajPointR.position.x = 0.0;
//    trajPointR.position.y = -0.0626366231343;
//    trajPointR.position.z = 0.0;
//    trajPointR.orientation.x = -1.16587645216e-06;
//    trajPointR.orientation.y = 3.03336607205e-06;
//    trajPointR.orientation.z = 0.000763307697428;
//    trajPointR.orientation.w = 0.999999708675;
//    trajPointR.time = 0.0;
//    msgR.taskspace_trajectory_points.push_back(trajPointR);

//    trajPointR.unique_id = 42; //(int)ros::Time::now();
//    trajPointR.position.x = 0.4;
//    trajPointR.position.y = -0.0626366231343;
//    trajPointR.position.z = 0.0;
//    trajPointR.orientation.x = -1.16587645216e-06;
//    trajPointR.orientation.y = 3.03336607205e-06;
//    trajPointR.orientation.z = 0.000763307697428;
//    trajPointR.orientation.w = 0.999999708675;
//    trajPointR.time = 0.8;
//    msgR.taskspace_trajectory_points.push_back(trajPointR);

//    trajPointR.unique_id = 43; //(int)ros::Time::now();
//    trajPointR.position.x = 0.4;
//    trajPointR.position.y = -0.0626366231343;
//    trajPointR.position.z = 0.0;
//    trajPointR.orientation.x = -1.16587645216e-06;
//    trajPointR.orientation.y = 3.03336607205e-06;
//    trajPointR.orientation.z = 0.000763307697428;
//    trajPointR.orientation.w = 0.999999708675;
//    trajPointR.time = 3.1;
//    msgR.taskspace_trajectory_points.push_back(trajPointR);


//    msgL.execution_mode=0;
//    msgL.robot_side = LEFT;
//    msgL.previous_message_id = 0;
//    msgL.taskspace_trajectory_points.clear();
//    msgL.unique_id = 4;

//    trajPointL.unique_id = 12; //(int)ros::Time::now();
//    trajPointL.time = 0.4; // reach instantenously
//    trajPointL.position.x = 0.0;
//    trajPointL.position.y = 0.118900228121;
//    trajPointL.position.z = 0.0;
//    trajPointL.orientation.x =  9.79238581203e-06;
//    trajPointL.orientation.y = 1.5494878979e-05;
//    trajPointL.orientation.z = -0.00103408687569;
//    trajPointL.orientation.w =  0.999999465164;

//    msgL.taskspace_trajectory_points.push_back(trajPointL);

//    trajPointL.unique_id = 11; //(int)ros::Time::now();
//    trajPointL.position.x = 0.4;
//    trajPointL.position.y = 0.118900228121;
//    trajPointL.position.z = 0.0;
//    trajPointL.orientation.x =  9.79238581203e-06;
//    trajPointL.orientation.y = 1.5494878979e-05;
//    trajPointL.orientation.z = -0.00103408687569;
//    trajPointL.orientation.w =  0.999999465164;
//    trajPointL.time = 1.6; // reach instantenously

//    msgL.taskspace_trajectory_points.push_back(trajPointL);


//   // ROS_INFO("published msg");
//    pub.publish(msgR);
//    ros::Duration(0.25).sleep();
//    pub.publish(msgL);
//}
