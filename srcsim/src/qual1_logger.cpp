#include <ros/ros.h>
#include <srcsim_msgs/groundTruth.h>
#include <led_detector/LedPositionColor.h>
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

/**
 * @brief The ConsoleFramePublisher class This class publishes console frame and groundTruth frame for led that is currently ON.
 *
 * This code should self destruct itself before end of qualifiers.
 */

class ConsoleFramePublisher{
public:
    ConsoleFramePublisher(ros::NodeHandle &nh);
    void GroundTruthCB(const srcsim_msgs::groundTruth &msg);
private:
    void publishStaticFrame(const ros::TimerEvent& e);
    ros::Subscriber groundTruthSub_;
    srcsim_msgs::groundTruth currentGroundTruth_;
    ros::Timer timer_;
};

ConsoleFramePublisher::ConsoleFramePublisher(ros::NodeHandle &nh)  {
    groundTruthSub_ = nh.subscribe("/srcsim/qual1/groundTruth",100,&ConsoleFramePublisher::GroundTruthCB,this);
    // Start the timer that will trigger the processing loop (publishStaticFrame)
    timer_ = nh.createTimer(ros::Duration(0.05), &ConsoleFramePublisher::publishStaticFrame, this);
}

void ConsoleFramePublisher::GroundTruthCB(const srcsim_msgs::groundTruth &msg)  {
    currentGroundTruth_ = msg;
}

void ConsoleFramePublisher::publishStaticFrame(const ros::TimerEvent& e){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(3, 0, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, -1.570796);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "console"));

    if (!currentGroundTruth_.data.empty()){
        transform.setOrigin( tf::Vector3(currentGroundTruth_.dataPose[0], currentGroundTruth_.dataPose[1], currentGroundTruth_.dataPose[2]) );
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "console", "groundTruth"));
    }
}

class logger
{
private:

    ros::Subscriber ground_truth,actual_data; // Subscriber
    void actual_data_callback(const  led_detector::LedPositionColor &msg);
    std::string file_name;
    std::ofstream csvfile;
    tf::TransformListener tlistener;
    ConsoleFramePublisher* consoleFrame;

public:
    logger(ros::NodeHandle nh);
    ~logger();
    srcsim_msgs::groundTruth groundtruth_msg_;  //message
    led_detector::LedPositionColor led_pos_msg_;

};

logger::logger(ros::NodeHandle nh)
{
    consoleFrame = new ConsoleFramePublisher(nh);
    //Subscriber Initializer
    actual_data = nh.subscribe("/detect/light/rgbxyz",100,&logger::actual_data_callback,this);
    file_name.assign(ros::package::getPath("srcsim"));
    file_name.append("/measurement_log.csv");
    if (!std::ifstream(file_name)) {
        csvfile.open(file_name,std::ios::app);
        csvfile<<"actual_x, actual_y, actual_z, measured_x, measured_y, measured_z"<<std::endl;
    }
}

logger::~logger(){
    delete consoleFrame;
}


void logger::actual_data_callback(const led_detector::LedPositionColor &msg) // Callback for actual data subscriber
{
    tf::StampedTransform groundTruthTansform;
    try{
        tlistener.lookupTransform("head","groundTruth", ros::Time(0), groundTruthTansform);
        groundtruth_msg_.dataPose[0] = groundTruthTansform.getOrigin().getX();
        groundtruth_msg_.dataPose[1] = groundTruthTansform.getOrigin().getY();
        groundtruth_msg_.dataPose[2] = groundTruthTansform.getOrigin().getZ();
    }
    catch(tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        groundtruth_msg_.dataPose[0] = 0;
        groundtruth_msg_.dataPose[1] = 0;
        groundtruth_msg_.dataPose[2] = 0;

    }

    led_pos_msg_= msg;


    std::cout<<"Ground Truth- x:"<<groundtruth_msg_.dataPose[0]<<" y:"<<groundtruth_msg_.dataPose[1]<<" z:"<<groundtruth_msg_.dataPose[2]<<std::endl;
    std::cout<<"Measured Val- x:"<<led_pos_msg_.position.x<<" y:"<<led_pos_msg_.position.y<<" z:"<<led_pos_msg_.position.z<<std::endl;
    //        std::stringstream ss;
    //        ss<<"Measured Value :"<< led_pos_msg_.position;
    //        ROS_INFO(ss.str().c_str());

    csvfile.open(file_name,std::ios::app);
    //actual_x, actual_y, actual_z, measured_x, measured_y, measured_z
    csvfile<<groundtruth_msg_.dataPose[0]<<","<<groundtruth_msg_.dataPose[1]<<","<<groundtruth_msg_.dataPose[2]<<","
                                        <<led_pos_msg_.position.x<<","<<led_pos_msg_.position.y<<","<<led_pos_msg_.position.z<<std::endl;
    csvfile.close();

}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "Qual1Log");
    ros::NodeHandle nh;
    logger data_log(nh);


    while (ros::ok())
    {

        ros::spin();
    }

    // csvfile.close();
    return 0;

}
