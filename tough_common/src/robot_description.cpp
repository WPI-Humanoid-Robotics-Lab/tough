#include "tough_common/robot_description.h"

#include <algorithm>
#include <string>
#include <cctype>

// The following function to find substring is copied from stack overflow
// Try to find in the Haystack the Needle - ignore case
bool findSubStringIC(const std::string & strHaystack, const std::string & strNeedle)
{
  auto it = std::search(
    strHaystack.begin(), strHaystack.end(),
    strNeedle.begin(),   strNeedle.end(),
    [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
  );
  return (it != strHaystack.end() );
}

// define static variables
RobotDescription* RobotDescription::object = nullptr;

RobotDescription* RobotDescription::getRobotDescription(ros::NodeHandle nh, std::string urdf_param)
{
    if(RobotDescription::object == nullptr){
        static RobotDescription obj(nh, urdf_param);
        object = &obj;
    }
    return object;
}

RobotDescription::RobotDescription(ros::NodeHandle nh, std::string urdf_param)
{
    param_left_arm_joint_names_  =  TOUGH_COMMON_NAMES::LEFT_ARM_JOINT_NAMES_PARAM ;
    param_right_arm_joint_names_ =  TOUGH_COMMON_NAMES::RIGHT_ARM_JOINT_NAMES_PARAM;
    param_left_foot_frame_name_  =  TOUGH_COMMON_NAMES::LEFT_FOOT_FRAME_NAME_PARAM ;
    param_right_foot_frame_name_ =  TOUGH_COMMON_NAMES::RIGHT_FOOT_FRAME_NAME_PARAM;

    if(!nh.getParam(TOUGH_COMMON_NAMES::ROBOT_NAME_PARAM, robot_name_)){
        ROS_ERROR("Could not read robot_name");
    }

    ROS_INFO("Robot Name : %s", robot_name_.c_str());
    std::string prefix = "/ihmc_ros/" + robot_name_ + "/";

    std::string robot_xml;
    urdf_param = "/"+robot_name_+urdf_param;
    if(!nh.getParam(urdf_param, robot_xml) || !model_.initString(robot_xml)){
        ROS_ERROR("Could not read the robot_description");
        return;
    }
    if (robot_name_ == ""){
        robot_name_.assign(model_.getName());
    }

    param_left_arm_joint_names_.insert(0, prefix);
    param_right_arm_joint_names_.insert(0, prefix);
    param_left_foot_frame_name_.insert(0, prefix);
    param_right_foot_frame_name_.insert(0, prefix);


    if(!(nh.getParam(param_left_arm_joint_names_, left_arm_joint_names_  ) &&
         nh.getParam(param_right_arm_joint_names_, right_arm_joint_names_) &&
         nh.getParam(param_left_foot_frame_name_, left_foot_frame_name_  ) &&
         nh.getParam(param_right_foot_frame_name_, right_foot_frame_name_) )) {

        ROS_ERROR("Could not read the joint names from parameter server");
        return;
    }


    // get a vector of all links
    model_.getLinks(links_);

    /*  There's a better way of doing this as param server already has the joint names
         // get a vector of all joints
        for (std::map<std::string,boost::shared_ptr<urdf::Joint> >::const_iterator joint = model_.joints_.begin();joint!= model_.joints_.end(); joint++)
        {
          joints_.push_back(joint->second);
        }


        // populate all the required joint limits
        for( auto joint : joints_) {
            std::cout<<joint->name<<std::endl;
        }
    */


    // set all the required frame names -- inefficient but works for now. important frame names should be on parameter server
    for( auto link : links_) {
        // assuming that pelvis frame will be the one with least length
        if(findSubStringIC(link->name, "pelvis")
           && (link->name.length() < PELVIS_TF.length() || PELVIS_TF.empty())){
            PELVIS_TF = link->name;
        }
    }

//    ROS_INFO("LEFT ARM");
    for (auto joint_name : left_arm_joint_names_){
        float l_limit = model_.joints_[joint_name]->limits->lower;
        float u_limit = model_.joints_[joint_name]->limits->upper;
        left_arm_joint_limits_.push_back({l_limit, u_limit});
        left_arm_frame_names_.push_back(model_.joints_[joint_name]->child_link_name);
//        ROS_INFO("\nFrame : %s \nJoint : %s \nLimits : <%0.4f , %0.4f>\n-------------\n", (left_arm_frame_names_.end()-1)->c_str(), joint_name.c_str(), l_limit, u_limit);
    }

    L_PALM_TF = *(left_arm_frame_names_.end()-1);
    TORSO_TF = model_.joints_[left_arm_joint_names_[0]]->parent_link_name;

    //    ROS_INFO("RIGHT ARM");
    for (auto joint_name : right_arm_joint_names_){
        float l_limit = model_.joints_[joint_name]->limits->lower;
        float u_limit = model_.joints_[joint_name]->limits->upper;
        right_arm_joint_limits_.push_back({l_limit, u_limit});
        right_arm_frame_names_.push_back(model_.joints_[joint_name]->child_link_name);
//        ROS_INFO("\nFrame : %s \nJoint : %s \nLimits : <%0.4f , %0.4f>\n-------------\n", (right_arm_frame_names_.end()-1)->c_str(), joint_name.c_str(), l_limit, u_limit);
    }

    R_PALM_TF = *(right_arm_frame_names_.end()-1);


    /* With 0.11 version of open-robotics-software, the foot offset is not handled on JAVA side
     * This causes footsteps to be at a height from ground. should this be fixed on JAVA side?
     */

    if(robot_name_ == "atlas"){
        R_END_EFFECTOR_TF = "r_palm";
        L_END_EFFECTOR_TF = "l_palm";
        number_of_neck_joints_ = 1;
        foot_frame_offset_ = 0.085;
    }
    else if (robot_name_ == "valkyrie"){
        R_END_EFFECTOR_TF = "rightMiddleFingerPitch1Link";
        L_END_EFFECTOR_TF = "leftMiddleFingerPitch1Link";
        number_of_neck_joints_ = 3;
        foot_frame_offset_ = 0.102;
    }

    updateFrameHash();

    ROS_INFO("Left foot frame : %s",  left_foot_frame_name_.c_str());
    ROS_INFO("Right foot frame : %s", right_foot_frame_name_.c_str());
    ROS_INFO("Pelvis Frame : %s", PELVIS_TF.c_str());
    ROS_INFO("Torso Frame : %s", TORSO_TF.c_str());
    ROS_INFO("Right Palm Frame : %s", R_PALM_TF.c_str());
    ROS_INFO("Left Palm Frame : %s", L_PALM_TF.c_str());
}

RobotDescription::~RobotDescription()
{

}

double RobotDescription::getFootFrameOffset() const
{
    return foot_frame_offset_;
}

int RobotDescription::getRightSoleFrameHash() const
{
    return RIGHT_SOLE_FRAME_HASH_;
}

int RobotDescription::getLeftSoleFrameHash() const
{
    return LEFT_SOLE_FRAME_HASH_;
}

int RobotDescription::getCenterOfMassFrameHash() const
{
    return CENTER_OF_MASS_FRAME_HASH_;
}

int RobotDescription::getChestFrameHash() const
{
    return CHEST_FRAME_HASH_;
}

int RobotDescription::getPelvisFrameHash() const
{
    return PELVIS_FRAME_HASH_;
}

int RobotDescription::getPelvisZUPFrameHash() const
{
    return PELVIS_ZUP_FRAME_HASH_;
}

int RobotDescription::getMidFeetZUPFrameHash() const
{
    return MIDFEET_ZUP_FRAME_HASH_;
}

int RobotDescription::getNumberOfNeckJoints() const
{
    return number_of_neck_joints_;
}

void RobotDescription::setNumberOfNeckJoints(int numberOfNeckJoints)
{
    number_of_neck_joints_ = numberOfNeckJoints;
}

bool RobotDescription::updateFrameHash()
{
    MIDFEET_ZUP_FRAME_HASH_ = -100;
    PELVIS_ZUP_FRAME_HASH_  = -101;
    PELVIS_FRAME_HASH_      = -102;
    CHEST_FRAME_HASH_       = -103;
    CENTER_OF_MASS_FRAME_HASH_ = -104;
    LEFT_SOLE_FRAME_HASH_   = -105;
    RIGHT_SOLE_FRAME_HASH_  = -106;
}


const std::string RobotDescription::getRightPalmFrame() const
{
    return R_PALM_TF;
}

const std::string RobotDescription::getRightEEFrame() const
{
    return R_END_EFFECTOR_TF;
}

const std::string RobotDescription::getLeftEEFrame() const
{
    return L_END_EFFECTOR_TF;
}

const std::string RobotDescription::getRobotName() const
{
    return robot_name_;
}

void RobotDescription::setRightPalmFrame(const std::string &value)
{
    R_PALM_TF = value;
}

const std::string RobotDescription::getLeftPalmFrame() const
{
    return L_PALM_TF;
}

void RobotDescription::setLeftPalmFrame(const std::string &value)
{
    L_PALM_TF = value;
}

void RobotDescription::getRightArmJointLimits(std::vector<std::pair<double, double> > &right_arm_joint_limits) const
{
    right_arm_joint_limits = right_arm_joint_limits_;
}

void RobotDescription::setRightArmJointLimits(const std::vector<std::pair<double, double> > &right_arm_joint_limits)
{
    right_arm_joint_limits_ = right_arm_joint_limits;
}

void RobotDescription::getLeftArmJointLimits(std::vector<std::pair<double, double> > &left_arm_joint_limits) const
{
    left_arm_joint_limits = left_arm_joint_limits_;
}

void RobotDescription::setLeftArmJointLimits(const std::vector<std::pair<double, double> > &left_arm_joint_limits)
{
    left_arm_joint_limits_ = left_arm_joint_limits;
}

const std::string RobotDescription::getRightFootFrameName() const
{
    return right_foot_frame_name_;
}

void RobotDescription::setRightFootFrameName(const std::string &right_foot_frame_name)
{
    right_foot_frame_name_ = right_foot_frame_name;
}

const std::string RobotDescription::getLeftFootFrameName() const
{
    return left_foot_frame_name_;
}

void RobotDescription::setLeftFootFrameName(const std::string &left_foot_frame_name)
{
    left_foot_frame_name_ = left_foot_frame_name;
}

void RobotDescription::getRightArmFrameNames(std::vector<std::string> &right_arm_frame_names) const
{
    right_arm_frame_names = right_arm_frame_names_;
}

void RobotDescription::setRightArmFrameNames(const std::vector<std::string> &right_arm_frame_names)
{
    right_arm_frame_names_ = right_arm_frame_names;
}

void RobotDescription::getLeftArmFrameNames(std::vector<std::string> &left_arm_frame_names) const
{
    left_arm_frame_names = left_arm_frame_names_;
}

void RobotDescription::setLeftArmFrameNames(const std::vector<std::string> &left_arm_frame_names)
{
    left_arm_frame_names_ = left_arm_frame_names;
}

void RobotDescription::getRightArmJointNames(std::vector<std::string> &right_arm_joint_names) const
{
    right_arm_joint_names = right_arm_joint_names_;
}

void RobotDescription::setRightArmJointNames(const std::vector<std::string> &right_arm_joint_names)
{
    right_arm_joint_names_ = right_arm_joint_names;
}

void RobotDescription::getLeftArmJointNames(std::vector<std::string> &left_arm_joint_names) const
{
    left_arm_joint_names = left_arm_joint_names_;
}

void RobotDescription::setLeftArmJointNames(const std::vector<std::string> &left_arm_joint_names)
{
    left_arm_joint_names_ = left_arm_joint_names;
}

const std::string RobotDescription::getTorsoFrame() const
{
    return TORSO_TF;
}

void RobotDescription::setTorsoFrame(const std::string &value)
{
    TORSO_TF = value;
}

const std::string RobotDescription::getPelvisFrame() const
{
    return PELVIS_TF;
}

const std::string RobotDescription::getWorldFrame() const
{
    return "world";
}

void RobotDescription::setPelvisFrame(const std::string &value)
{
    PELVIS_TF = value;
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_urdf");
    ros::NodeHandle nh;
    RobotDescription* robot = RobotDescription::getRobotDescription(nh);
    return 0;
}
