#include <algorithm>
#include <val_controllers/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <fstream>
#include "ros/package.h"
#include "val_moveit_planners/val_cartesian_planner.h"
#include "val_controllers/val_wholebody_manipulation.h"
#include "val_controllers/val_pelvis_navigation.h"
#include <navigation_common/fall_detector.h>

float COM_X, COM_Y, COM_Z;
bool READ_COM=false;
const int NUMBER_OF_ITERATIONS=10;

void executeTrajectorySingleStage(wholebodyManipulation* wholebody_controller,cartesianPlanner* traj_planner, geometry_msgs::Pose &pose){

    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory traj;
    waypoints.push_back(pose);
    traj_planner->getTrajFromCartPoints(waypoints, traj,false);
    wholebody_controller->compileMsg(armSide::RIGHT, traj.joint_trajectory);
    ros::Duration(5.0).sleep();
    return;
}

void executeTrajectory2Stage(wholebodyManipulation* wholebody_controller,armTrajectory* arm_traj, cartesianPlanner* traj_planner, geometry_msgs::Pose &pose){

    arm_traj->moveArmInTaskSpace(armSide::RIGHT, pose, 3.0);
    ros::Duration(3.0).sleep();
    std::vector<geometry_msgs::Pose> waypoints;
    moveit_msgs::RobotTrajectory traj;
    waypoints.push_back(pose);
    traj_planner->getTrajFromCartPoints(waypoints, traj,false);
    wholebody_controller->compileMsg(armSide::RIGHT, traj.joint_trajectory);
    ros::Duration(3.0).sleep();
    return;
}

void logToFile(){

}

void readCoMPosition(visualization_msgs::Marker::Ptr msg){
    if(READ_COM){
        READ_COM = false;
        COM_X = msg->pose.position.x;
        COM_Y = msg->pose.position.y;
        COM_Z = msg->pose.position.z;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "com_experiment");
    ros::NodeHandle nh;

    ros::Subscriber com_reader = nh.subscribe("actualCOM",5,readCoMPosition);
    ros::AsyncSpinner spinner(1);
    spinner.start();


    armTrajectory armTraj(nh);
    chestTrajectory chestTraj(nh);
    pelvisTrajectory pelvisTraj(nh);

    wholebodyManipulation* wholebody_controller = new wholebodyManipulation(nh);
    cartesianPlanner* traj_planner = new cartesianPlanner("rightPalm", VAL_COMMON_NAMES::WORLD_TF); //rightMiddleFingerGroup

    fallDetector fall_detector(nh, "leftFoot", "pelvis", "world");
     if (fall_detector.isRobotFallen()){
         return -1;
     }
    pelvisTraj.controlPelvisHeight(0.9);
    ros::Duration(1.5).sleep();
    chestTraj.controlChest(2,2,2);
    ros::Duration(1).sleep();

    armTraj.moveToDefaultPose(armSide::LEFT);
    ros::Duration(0.3).sleep();
    armTraj.moveToDefaultPose(armSide::RIGHT);
    ros::Duration(1).sleep();

    geometry_msgs::Pose goal;
    int iterator;
    int runID;


    std::string filename = ros::package::getPath("test") + "/log/singleStageResults.csv";
    std::ofstream logger;
    struct stat buffer;
    if (stat (filename.c_str(), &buffer) != 0){
        logger.open(filename.c_str());
        logger << "runID, iterator, position.x, position.y, position.x, orientation.x, orientation.y, orientation.z,orientation.w, CoM.x, CoM.y, CoM.z, CoM.x_ddot, CoM.y_ddot, success_flag"<<std::endl;

        goal.position.x =  0.28;
        goal.position.y = -0.28;
        goal.position.z =  0.90;

        goal.orientation.x = -0.576;
        goal.orientation.y =  0.397;
        goal.orientation.z =  0.632;
        goal.orientation.w =  0.332;


        iterator = 0;
        runID = 0;

    }
    else{
        // read last goal position, iterator, and runID from log file. last line is always blank!
        std::ifstream fileReader(filename.c_str());
        std::string line1, line2;
        while (getline (fileReader,line1)){
            if(line1.empty()) continue;
            else line2.assign(line1);
        }
        std::replace(line2.begin(),line2.end(),',',' ');
        std::stringstream ss(line2);
        ss >> runID >> iterator >> goal.position.x >> goal.position.y
           >> goal.position.z >> goal.orientation.x >> goal.orientation.y
           >> goal.orientation.z >> goal.orientation.w;
        fileReader.close();
        ++runID;
        ++iterator;
        logger.open(filename.c_str(), std::fstream::app);

    }


    for(;goal.position.x < 1.1;goal.position.x += 0.1){
        for(;goal.position.y > -1.1;goal.position.y -= 0.1){
            for(;goal.position.z < 1.1;goal.position.z += 0.1){

                for (; iterator <= NUMBER_OF_ITERATIONS; ++iterator){
                    std::cout<<"goal position :"<<goal.position.x <<", "<<goal.position.y <<", "<<goal.position.z <<"Iterator :"<<iterator<<" RunID:"<<runID<<std::endl;
                    //execute trajectory in single stage
                    //            executeTrajectorySingleStage(wholebody_controller, traj_planner, goal);
                    executeTrajectory2Stage(wholebody_controller,&armTraj,traj_planner,goal);

                    READ_COM = true;
                    ros::Duration(1.0).sleep();

                    //log the goal pose, COM location, and run ID
                    logger << runID<<",";
                    logger << iterator<<",";
                    logger << goal.position.x<<",";
                    logger << goal.position.y<<",";
                    logger << goal.position.z<<",";
                    logger << goal.orientation.x<<",";
                    logger << goal.orientation.y<<",";
                    logger << goal.orientation.z<<",";
                    logger << goal.orientation.w<<",";
                    logger << COM_X <<","<< COM_Y <<","<<COM_Z<<",";

                    //check if robot is standing
                    if (fall_detector.isRobotFallen()){
                        logger << "false" << std::endl;
                        logger.close();
                        return -1;
                    }
                    logger << "true" << std::endl;

                    //reset to default pose
                    pelvisTraj.controlPelvisHeight(0.9);
                    ros::Duration(1.5).sleep();
                    chestTraj.controlChest(2,2,2);
                    ros::Duration(1).sleep();

                    armTraj.moveToDefaultPose(armSide::LEFT);
                    ros::Duration(0.3).sleep();
                    armTraj.moveToDefaultPose(armSide::RIGHT);
                    ros::Duration(3).sleep();

                    ++runID;
                }

                // change the goal pose
                iterator=0;
            }
            goal.position.z = 0.90;
        }
        goal.position.y = -0.28;
    }
    logger.close();
    return 0;
}


