#include <ros/ros.h>
#include <tough_perception_common/ArucoDetector.h>

void printPoseMessages(std::string id,geometry_msgs::PoseStampedPtr &p)
{
    std::cout   << "ID:"<< id << " | "
                << p->pose.position.x << " " 
                << p->pose.position.y << " "
                << p->pose.position.z << std::endl;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_multisense_image");
    ros::NodeHandle nh;

    tough_perception::ArucoDetector detector(nh);

    std::map<std::string,geometry_msgs::PoseStampedPtr> detected_objects;

    std::map<std::string, std::string> object_ids;
    object_ids["10"] = "Gantry Front";
    object_ids["15"] = "Gantry Back";

    for(int i=0;i<100;i++)
    {
        detector.getDetectedObjects(detected_objects);

        for (std::map<std::string, geometry_msgs::PoseStampedPtr>::iterator it = detected_objects.begin(); it != detected_objects.end(); it++ )
        {
            printPoseMessages(object_ids[it->first],it->second);        
        }
    }
    
}