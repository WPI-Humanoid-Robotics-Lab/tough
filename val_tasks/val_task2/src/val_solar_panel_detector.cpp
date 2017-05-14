#include "val_task2/val_solar_panel_detector.h"
SolarPanelDetect::SolarPanelDetect(ros::NodeHandle nh, geometry_msgs::Pose rover_loc, bool isroverRight)
{
    pcl_sub =  nh.subscribe("/field/assembled_cloud2", 10, &SolarPanelDetect::cloudCB, this);;
    pcl_filtered_pub = nh.advertise<sensor_msgs::PointCloud2>("/val_solar_panel/cloud2", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "/val_solar_panel/visualization_marker", 1 );
    rover_loc_  =rover_loc;
    isroverRight_ = isroverRight;
//    ROS_INFO("rover loc const %f %f right: %d",rover_loc_.position.x,rover_loc_.position.y,(int)isroverRight_);
    setRoverTheta();
    setXoffset();
//    ROS_INFO("in const");
}

SolarPanelDetect::~SolarPanelDetect()
{
    pcl_sub.shutdown();
    if(robot_state_ != nullptr)         delete robot_state_;
}
bool SolarPanelDetect::getDetections(std::vector<geometry_msgs::Pose> &ret_val)
{
    ret_val.clear();
    ret_val = detections_;
    return !ret_val.empty();

}

int SolarPanelDetect::getDetectionTries() const
{
    return detection_tries_;

}

void SolarPanelDetect::getXoffset(float &minX, float &maxX)
{
    minX = min_x;
    maxX = max_x;
}

void SolarPanelDetect::setXoffset(float minX, float maxX)
{
    //Note: the func arg is relative to rover loc
    float slope =tan(rover_theta);
    ROS_INFO("setting X offset");

    min_x = minX;
    max_x = maxX;

}


void SolarPanelDetect::setRoverTheta()
{
    tf::Quaternion quat(rover_loc_.orientation.x,rover_loc_.orientation.y,rover_loc_.orientation.z,rover_loc_.orientation.w);
    tf::Matrix3x3 rotation(quat);
    tfScalar roll,pitch,yaw;
    rotation.getRPY(roll,pitch,yaw);
    rover_theta = yaw;
    ROS_INFO("angle %.2f",rover_theta);
    if (isroverRight_)
    {
        rover_theta-=1.5708;
    }
    else
    {
        rover_theta+=1.5708;
    }

}

void SolarPanelDetect::cloudCB(const sensor_msgs::PointCloud2::Ptr &input)
{
    if (input->data.empty()){
        return;
    }
    ++detection_tries_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 output;
    pcl::fromROSMsg(*input, *cloud);

    ROS_INFO("in clud Cb");

    transformCloud(cloud,true);
    PassThroughFilter(cloud);

//    transformCloud(cloud,false);

    ROS_INFO("pub size cloud ",(int)cloud->points.size());










    pcl::toROSMsg(*cloud,output);
    output.header.frame_id="world";
    pcl_filtered_pub.publish(output);

}
void SolarPanelDetect::transformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, bool isinverse)
{
    ROS_INFO("trasnforming cloud");
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << rover_loc_.position.x,rover_loc_.position.y,rover_loc_.position.z;

//    transform.translation() << 0,0,0;

    transform.rotate (Eigen::AngleAxisf (rover_theta, Eigen::Vector3f::UnitZ()));
    if (isinverse)
    {
        transform = transform.inverse();
    }
    pcl::transformPointCloud (*cloud, *cloud, transform);
    ROS_INFO("trasnforming cloud");
}

void SolarPanelDetect::PassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{


    float min_x,min_y,max_x,max_y;
    min_x = 0;
    max_x = 1;
    min_y = -1.5;
    max_y =  1.5;

    geometry_msgs::Point pt_in,pt_out;
/*
    // transforming pts from pelvis to world frame
    pt_in.x = solar_pass_x_min;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    min_x = pt_out.x;
    pt_in.x = solar_pass_x_max;
    pt_in.y =  0;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    max_x = pt_out.x;
    pt_in.x =  0;
    pt_in.y = solar_pass_y_min;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    min_y = pt_out.y;
    pt_in.x =  0;
    pt_in.y = solar_pass_y_max;
    pt_in.z =  0;
    robot_state_->transformPoint(pt_in,pt_out,VAL_COMMON_NAMES::PELVIS_TF);
    max_y = pt_out.y;
*/

    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(min_x,max_x);
    pass_x.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(min_y,max_y);
    pass_y.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.8,3);
    pass_z.filter(*cloud);

}
