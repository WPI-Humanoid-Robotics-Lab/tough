#include "src_task2/val_solar_detection.h"
#include "src_task2/val_rover_detection.h"

#include "tough_common/val_common_names.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "solar_array_detection");
    ros::NodeHandle nh;
    geometry_msgs::Pose2D rover_loc;
    std::vector<std::vector<geometry_msgs::Pose>> rover_poses;
    bool isroverRight;
    int NUM_SAMPLES=2;
    ros::Publisher goalPub = nh.advertise<geometry_msgs::PoseStamped>("/valkyrie/goal",1);

    ros::Rate loop1(1);

    if(true) //to destruct the rover_obj
    {
        RoverDetector rover_obj(nh);
        while(ros::ok())
        {

            rover_obj.getDetections(rover_poses);
            if(rover_poses.size())
            {
                geometry_msgs::Pose rover_loc_3d;
                rover_loc_3d = rover_poses[rover_poses.size()-1][2];
                rover_loc.x = rover_loc_3d.position.x;
                rover_loc.y = rover_loc_3d.position.y;
                rover_loc.theta = tf::getYaw(rover_loc_3d.orientation);
                ROVER_SIDE roverSide;
                if(rover_obj.getRoverSide(roverSide)){
                    isroverRight = roverSide == ROVER_SIDE::RIGHT;
                    break;
                }

            }
            ros::spinOnce();
            loop1.sleep();
        }
    }



    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    goal.pose = rover_poses[rover_poses.size() -1][2];
//    goalPub.publish(goal);
    //  std::cout <<obj.isRoverOnRight()<<std::endl;
    ROS_INFO("Reaching Rover ");
//    ros::Duration(40).sleep();


    SolarArrayDetector obj(nh,rover_loc,isroverRight);
    ros::Rate loop(1);
    std::vector<geometry_msgs::Pose> solar_array_poses;
    while(ros::ok()&& solar_array_poses.size() < NUM_SAMPLES){
        if(obj.getDetections(solar_array_poses))
            for (size_t i = 0; i < solar_array_poses.size(); ++i){
                //ROS_INFO_STREAM("x : "<<poses[i].position.x<<"y : "<<poses[i].position.y<<"z : "<<poses[i].position.z);
                //ROS_INFO_STREAM("x : "<<poses[i].orientation.x<<"y : "<<poses[i].orientation.y<<"z : "<<poses[i].orientation.z<<" w: "<<poses[i].orientation.w);
            }
        ros::spinOnce();
        loop.sleep();
    }
    goal.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    goal.pose = solar_array_poses[NUM_SAMPLES -1];
//    goalPub.publish(goal);
    //  std::cout <<obj.isRoverOnRight()<<std::endl;
    ROS_INFO("Reaching solar Array coarse");
    //  ros::Duration(20).sleep();
    return 0;
}
