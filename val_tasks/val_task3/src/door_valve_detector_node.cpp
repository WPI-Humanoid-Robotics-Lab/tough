#include "val_task3/door_valve_detector.h"
#include "val_common/val_common_names.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "door_valve_detector");
    ros::NodeHandle nh;

    std::vector<geometry_msgs::Point> valveCentres;
    DoorValvedetector detectionObj(nh);
    size_t retry_count = 1;

    // Do door detection stuff

    while (!detectionObj.getDetections(valveCentres) && retry_count < 4){

        ROS_INFO("Try Number : %d", retry_count);
        retry_count++;

        ros::Duration(3.0).sleep();
    }

    if(detectionObj.getDetections(valveCentres)){


        ROS_INFO_STREAM("Valve centre detected at x: " << valveCentres[0].x
                        << " y : " << valveCentres[0].y << " z : " << valveCentres[0].z);
    }

    else {

        ROS_INFO("Unable to detect the valve");
    }


    ros::spin();
    return 0;
}
