#include "val_task3/door_valve_detector.h"
#include "val_common/val_common_names.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "door_valve_detector");

    ros::NodeHandle nh;


    DoorValvedetector obj(nh);

    ros::Rate loop(1);

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
