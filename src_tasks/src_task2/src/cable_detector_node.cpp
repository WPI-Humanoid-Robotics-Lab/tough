#include <src_task2/cable_detector.h>
#include <src_task2/cable_task.h>
#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/chest_control_interface.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findCableDetector");
    ros::NodeHandle nh;
    ros::Publisher pub;
    ArmControlInterface armTraj_(nh);
    chestTrajectory chest_controller_(nh);
    ros::Rate loop(15);
    pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);
    int numIterations = 0;
    bool foundStandPos = false;
    geometry_msgs::Point CableLoc;
    geometry_msgs::Pose CablePose;
    tough_perception::MultisenseImage* ms_sensor = new tough_perception::MultisenseImage(nh);
    CableDetector c1(nh, ms_sensor);
    CableTask cable(nh);
    //    //c1.findCable(CableLoc);
    //    //while (!c1.findCable(CableLoc)){
    //    //while (){
    //    while(ros::ok()){
    //        c1.isCableinHand();
    //        //c1.findCable(CableLoc);
    //        //c1.findCable(CablePose);
    //            //while (!foundCable && numIterations < 20)
    //        //ROS_INFO("Cable location x:%f y:%f z:%f", CableLoc.x, CableLoc.y, CableLoc.z);
    //        //ROS_INFO("Cable location x:%f y:%f z:%f", CablePose.position.x, CablePose.position.y, CablePose.position.z);
    //        ros::spinOnce();
    //    }

    std::vector< std::vector<float> > armData;
    armData.push_back({0.2,0.2,0.2,0.2,0.2,0.2,0.2});


    char input;
    while(ros::ok)
    {
        armTraj_.moveArmJoints(RIGHT, armData,2.0f);
        chest_controller_.controlChest(0,0,0);
        ros::Duration(1).sleep();

        cout<<"enter preference \n";
        cout<<"r -rotate cable \n";
        cout<<"g -grab choke \n";
        cin>>input;

        c1.findCable(CablePose);
        ros::Duration(1).sleep();


        if(input == 'r')
        {
            cable.rotate_cable1(CablePose);
        }
        else if(input == 'g')
        {
            cable.grasp_choke(RIGHT,CablePose);
        }
        else
        {
            cable.drop_cable(RIGHT);
        }

        ros::spinOnce();
        ros::Duration(1).sleep();
    }



}

