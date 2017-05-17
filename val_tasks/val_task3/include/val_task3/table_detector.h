#ifndef TABLE_DETECTOR_H
#define TABLE_DETECTOR_H

#include <geometry_msgs/Point.h>

#include <perception_common/MultisenseImage.h>
#include <perception_common/MultisensePointCloud.h>
#include <perception_common/PointCloudHelper.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

class table_detector
{
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;

public:
    table_detector(ros::NodeHandle nh);

    ~table_detector();

};
#endif // TABLE_DETECTOR_H
