#ifndef TABLE_DETECTOR_H
#define TABLE_DETECTOR_H

#include <ros/ros.h>

#include <perception_common/MultisensePointCloud.h>

class table_detector {

    ros::NodeHandle nh_;
    src_perception::MultisensePointCloud point_cloud_combiner_;

public:
    table_detector(ros::NodeHandle nh);

    bool find_table();
};

#endif // TABLE_DETECTOR_H
