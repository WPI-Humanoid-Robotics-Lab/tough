#include <val_task2/cable_detector.h>
#include <val_task2/cable_task.h>
#include <val_controllers/val_arm_navigation.h>
#include <val_controllers/val_chest_navigation.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init (argc,argv,"findCableDetector");
    ros::NodeHandle nh;
    ros::Publisher pub;
    armTrajectory armTraj_(nh);
    chestTrajectory chest_controller_(nh);
    ros::Rate loop(15);
    pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1, true);
    int numIterations = 0;
    bool foundStandPos = false;
    geometry_msgs::Point CableLoc;
    geometry_msgs::Pose CablePose;
    src_perception::MultisenseImage* ms_sensor = new src_perception::MultisenseImage(nh);
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
            cable.rotate_cable(CablePose);
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

