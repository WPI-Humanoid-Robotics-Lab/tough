#include "val_task3/table_detector.h"

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findTableDetector");
    ros::NodeHandle nh;

    table_detector detector(nh);

    while (true) {
        detector.find_table();
    }

    ROS_INFO("Table detector started");
}

