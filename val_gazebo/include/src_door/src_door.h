#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <gazebo/transport/transport.hh>

using namespace gazebo;

class SRCDoor {
  public:
    SRCDoor();
    void run(int argc, char **argv);

  private:
    ros::Subscriber sub;
    transport::PublisherPtr pub;
    void callback(const std_msgs::Bool &msg);
};
