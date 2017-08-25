#include <humanoid_common/stability.h>
#include <humanoid_common/com.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <val_interface/val_JointState.h>


//////////
// test file for the com and stabiltiy criteria
//////////

class TestCOM {
public:
    TestCOM(ros::NodeHandle nh);
    ~TestCOM();
    void test(void);

protected:
    ros::NodeHandle nh_;
    ros::Publisher visualization_pub_, support_polygon_pub_, pcom_pub_, act_com_marker_;
    stability* Stability;
    com* COM;
    stability::FootSupport support_mode_;
    valJointState* joint;
};

TestCOM::TestCOM(ros::NodeHandle nh)
    : nh_(nh)
{
    COM = new com(nh, "pelvis", "rightFoot", "leftFoot", true);
    Stability = new stability(1, nh, "rightFoot", "pelvis", "rightFoot", "leftFoot");
    joint = new valJointState(nh);

    // create publisher and suscribers
    support_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("support_polygon", 10);
    pcom_pub_ = nh_.advertise<visualization_msgs::Marker>("projected_COM", 10);
    act_com_marker_ = nh_.advertise<visualization_msgs::Marker>("actualCOM", 10);

    support_mode_=stability::FootSupport::SUPPORT_DOUBLE;
}

TestCOM::~TestCOM(){
    delete COM;
    delete Stability;
}

void TestCOM::test(void){

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

    bool stable = Stability->isPoseStable(joint_positions_, support_mode_, normal_vector);

    // print info
    tf::Point com = Stability->getCOM();

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

    geometry_msgs::PolygonStamped support_polygon = Stability->getSupportPolygon();

//    std::cout<<support_polygon.polygon.points.size()<<"\n";

    std::vector<float> x,y;
    for (int i = 0; i < support_polygon.polygon.points.size(); ++i) {
           x.push_back(support_polygon.polygon.points[i].x);
           std::cout<<"x at "<<i<<" :"<<x[i]<<"\n";
           y.push_back(support_polygon.polygon.points[i].y);
           std::cout<<"y at "<<i<<" :"<<y[i]<<"\n";
    }

    float max_x = *std::max_element(std::begin(x),std::end(x));
    float max_y = *std::max_element(std::begin(y),std::end(y));

    std::cout<<max_x<<"\t"<<max_y<<"\n";

    float stability_margin = 200- (std::fabs(com.x())/max_x + std::fabs(com.y())/max_y)*50;

    if (stable)
    {
        ROS_INFO("COM inside polygon, pCOM at %f %f", com.x(), com.y());

        std::cout<<"stability margin is: "<<stability_margin<<" % \n";
    }
    else
    {
        ROS_ERROR("COM not inside polygon, pCOM at %f %f", com.x(), com.y());
    }
    support_polygon_pub_.publish(support_polygon);
    pcom_pub_.publish(Stability->getProjectedCOMMarker());
    act_com_marker_.publish(com_marker);
}

// test code
int main(int argc, char *argv[])
{

    ros::init(argc, argv, "test_com");
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    TestCOM T(nh);

    while(ros::ok())
    {
        T.test();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
