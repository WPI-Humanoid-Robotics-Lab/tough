#include "val_common/robot_description.h"

#include <algorithm>
#include <string>
#include <cctype>

/// Try to find in the Haystack the Needle - ignore case
bool findSubStringIC(const std::string & strHaystack, const std::string & strNeedle)
{
  auto it = std::search(
    strHaystack.begin(), strHaystack.end(),
    strNeedle.begin(),   strNeedle.end(),
    [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
  );
  return (it != strHaystack.end() );
}

RobotDescription::RobotDescription(ros::NodeHandle nh, std::string urdf_param)
{
    std::string robot_xml;

    if(!nh.getParam(urdf_param, robot_xml) || !model_.initString(robot_xml)){
        std::cout<<"Could not read the robot_description"<<std::endl;
        return;
    }
    robot_name_ = model_.getName();
    std::cout<<"Robot Name : "<<robot_name_<<std::endl;

    // get a vector of all links
    model_.getLinks(links_);

    // get a vector of all joints
    for (std::map<std::string,boost::shared_ptr<urdf::Joint> >::const_iterator joint = model_.joints_.begin();joint!= model_.joints_.end(); joint++)
    {
      joints_.push_back(joint->second);
    }

    // set all the required frame names
    for( auto link : links_) {
        // assuming that pelvis frame will be the one with least length
        if(findSubStringIC(link->name, "pelvis")
           && (link->name.length() < PELVIS_TF.length() || PELVIS_TF.empty())){
            PELVIS_TF = link->name;
        }

    }
    std::cout<< "Pelvis Frame : " << PELVIS_TF <<std::endl;

    // populate all the required joint limits
    for( auto joint : joints_) {
        std::cout<<joint->name<<std::endl;
    }

}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_urdf");
    ros::NodeHandle nh;
    RobotDescription robot(nh);
    return 0;
}
