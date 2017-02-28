#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;
using namespace boost::assign;

#define foreach BOOST_FOREACH


int main(int argc, char **argv) {
    ros::init(argc, argv, "send_events");

    ros::NodeHandle node;
    ros::Publisher eventPublisher = node.advertise<std_msgs::String>("/decision_making/FSMTask2/events", 1, false);

    int currentEvent = 0;
    vector<string> events;
    events += "FOUND","CLOSE_TO_OBJECT","GRABBED";

    ROS_INFO("Starting robot event publisher...");

    while (ros::ok()) {
        std_msgs::String event;
        event.data = events[currentEvent++];
        currentEvent %= events.size();

        eventPublisher.publish(event);
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    }
	return 0;
}
