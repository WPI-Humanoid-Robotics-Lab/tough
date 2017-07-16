#include <humanoid_common/stability.h>
#include <humanoid_common/com.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <val_interface/val_JointState.h>


//////////
// test file for the com and stabiltiy criteria
//////////

class testCOMStability {
public:
    testCOMStability(ros::NodeHandle nh);
    ~testCOMStability();
    void jointStateCb(const sensor_msgs::JointStateConstPtr& state);
    void test(void);

protected:
    ros::NodeHandle nh_;
    ros::Publisher visualization_pub_, support_polygon_pub_, pcom_pub_, act_com_marker_;
    stability* S;
    com* COM;
    stability::FootSupport support_mode_;
    valJointState* joint;
};

testCOMStability::testCOMStability(ros::NodeHandle nh)
    : nh_(nh)
{
    COM = new com(nh, "pelvis", "rightFoot", "leftFoot", true);
    S = new stability(1, nh, "rightFoot", "pelvis", "rightFoot", "leftFoot");
    joint = new valJointState(nh);

    // create publisher and suscribers
    support_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("support_polygon", 10);
    pcom_pub_ = nh_.advertise<visualization_msgs::Marker>("projected_COM", 10);
    act_com_marker_ = nh_.advertise<visualization_msgs::Marker>("actualCOM", 10);

    support_mode_=stability::SUPPORT_DOUBLE;
}

testCOMStability::~testCOMStability(){

}

void testCOMStability::test(void){

    // get joint positions from state message
    std::map<std::string, double> joint_positions_;

    for (unsigned int i=0; i<joint->getJointStates().name.size(); i++)
    {
        joint_positions_.insert(make_pair(joint->getJointStates().name[i], joint->getJointStates().position[i]));
        //std::cout << "name " << state->name[i] << " pos "<<  state->position[i] << std::endl;
    }

    tf::Vector3 normal_vector(0.0, 0.0, 1.0); // planar support
    // for testing, normal vector of arbitrary plane:
    normal_vector.normalize();

    //ROS_INFO("callback");
    bool stable = S->isPoseStable(joint_positions_, support_mode_, normal_vector);

    // print info
    tf::Point com = S->getCOM();

    tf::Transform tf_right_foot, tf_left_foot;
    tf::Point com_act;
    double mass;
    COM->computeCOM(joint_positions_, com_act, mass, tf_right_foot, tf_left_foot);

    visualization_msgs::Marker com_marker;
    com_marker.header.stamp = ros::Time::now();
    com_marker.header.frame_id = "pelvis";
    com_marker.action = visualization_msgs::Marker::ADD;
    com_marker.type = visualization_msgs::Marker::SPHERE;
    com_marker.pose.position.x = com_act.getX();
    com_marker.pose.position.y = com_act.getY();
    com_marker.pose.position.z = com_act.getZ();
    com_marker.scale.x = 0.05;
    com_marker.scale.y = 0.05;
    com_marker.scale.z = 0.01;
    com_marker.color.a = 1.0;
    com_marker.color.g = 0.0;
    com_marker.color.r = 0.0;
    com_marker.color.b = 1.0;

    if (stable)
      ROS_INFO("Pose is stable, pCOM at %f %f", com.x(), com.y());
    else
      ROS_INFO("Pose is NOT stable, pCOM at %f %f", com.x(), com.y());

    support_polygon_pub_.publish(S->getSupportPolygon());
    pcom_pub_.publish(S->getProjectedCOMMarker());
    act_com_marker_.publish(com_marker);
}

// test code
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "test_com");
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    testCOMStability T(nh);

    while(ros::ok())
    {
        T.test();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
