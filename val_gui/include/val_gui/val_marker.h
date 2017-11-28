#ifndef val_marker_h
#define val_marker_h

#include <ros/ros.h>
#include <string>
#include <tough_common/val_common_names.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


using namespace visualization_msgs;
using namespace std;
void make6DofMarker(std::string name,interactive_markers::InteractiveMarkerServer *i_marker_server,
                    void (*callback)(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &),
                    const tf::Vector3& position,geometry_msgs::Quaternion palm_rotation ,interactive_markers::MenuHandler *menu_handler);
InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );
Marker makeBox( InteractiveMarker &msg );
geometry_msgs::PoseStamped transform_pose_simple(const geometry_msgs::PoseStamped *from_pose, std::string to_frame);

class Val_Marker{

public:
  Val_Marker(std::string node_name, std::string marker_name, geometry_msgs::PoseStamped &start_position, void (*callback)(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &) );
  ~Val_Marker();
  void add_marker(std::string marker_name, geometry_msgs::PoseStamped start_position,void (*callback)(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &) );
private:
  interactive_markers::MenuHandler *menu_handler;
  interactive_markers::InteractiveMarkerServer *i_marker_server;

};

#endif
