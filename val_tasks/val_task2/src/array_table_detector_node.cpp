#include <val_task2/cable_detector.h>
#include <val_task2/array_table_detector.h>

int main(int argc, char** argv)
{
    ros::init (argc,argv,"array_table_detector");
    ros::NodeHandle nh;

    ros::Rate loop(0.25);
    int numIterations = 0;

    geometry_msgs::Point CableLoc;
    geometry_msgs::Pose StandLoc;
    CableDetector c1(nh);

    while (!c1.findCable(CableLoc)){
        ROS_INFO("Cable detection failed. retrying");
    }
    ArrayTableDetector atd(nh, CableLoc);
    while(ros::ok())
    //while (!foundCable && numIterations < 20)
    {


        std::vector<geometry_msgs::Pose> detections;
        if(atd.getDetections(detections))
        {
            StandLoc = detections.back();
            ROS_INFO("Robot Pose: x:%f y:%f z:%f", StandLoc.position.x, StandLoc.position.y, StandLoc.position.z);
        }
        ros::spinOnce();
        loop.sleep();
        numIterations++;
    }

}


