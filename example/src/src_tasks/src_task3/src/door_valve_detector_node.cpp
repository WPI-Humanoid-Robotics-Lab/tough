#include "src_task3/door_valve_detector.h"
#include "tough_common/val_common_names.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "door_valve_detector");
    ros::NodeHandle nh;

    std::vector<geometry_msgs::Pose> valveCentres;
    DoorValvedetector detectionObj(nh);
    size_t retry_count = 0;
    ros::Rate loop(0.3);
    // Do door detection stuff
    bool success = false;
    while (retry_count++ < 5 ){
        success = detectionObj.getDetections(valveCentres);
        if (success){
            break;
        }
        ros::spinOnce();
        loop.sleep();
    }


    if(success){

        ROS_INFO_STREAM("Valve centre detected at x: " << valveCentres[0].position.x
                << " y : " << valveCentres[0].position.y << " z : " << valveCentres[0].position.z);
    }

    else {

        ROS_INFO("Unable to detect the valve");
    }





    return 0;
}
