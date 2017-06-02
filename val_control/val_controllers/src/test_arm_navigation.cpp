#include <val_controllers/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_arm_navigation");
    ros::NodeHandle nh;
    ROS_INFO("Moving the arms");
    armTrajectory armTraj(nh);
    while(ros::ok())
    {
        std::cout<<"Arm motion\n"
                   "1. Move a joint using joint number\n"
                   "2. Move to a 3D point in task space\n"
                   "3. Move to a 6D point in task\n"
                   "4. Move to a specific configuration\n"
                   "5. Run Demo\n"
                   "Select a motion:";
        int choice, inputSide;
        std::cin>>choice;
        switch (choice) {
        case 1:
        {
            std::cout << "Enter <side> <jointNumber> <JointAngle> :";
            int   jointNumber;
            float jointAngle;
            std::cin>>inputSide>>jointNumber>>jointAngle;

            armSide side;
            if(inputSide == 0){
                side = LEFT;
            } else {
                side = RIGHT;
            }
            armTraj.moveArmJoint(side, jointNumber,jointAngle);
            break;
        }
        case 2:{
            std::cout << "Enter <side> <x> <y> <z> :";
            geometry_msgs::Pose pt;
            std::cin>>inputSide>> pt.position.x>>pt.position.y >>pt.position.z;
            pt.orientation.w = 1.0;
            armSide side;
            if(inputSide == 0){
                side = LEFT;
            } else {
                side = RIGHT;
            }
            armTraj.moveArmInTaskSpace(side, pt, 3.0);
            break;
        }
        case 3:{
            std::cout << "Enter <side> <x> <y> <z> <ax> <ay> <az> <aw>:";
            geometry_msgs::Pose pt;
            std::cin>>inputSide>>pt.position.x>>pt.position.y >>pt.position.z>> pt.orientation.x >>pt.orientation.y >> pt.orientation.z >>pt.orientation.w;
            armSide side;
            if(inputSide == 0){
                side = LEFT;
            } else {
                side = RIGHT;
            }
            armTraj.moveArmInTaskSpace(side, pt, 3.0);
            break;
        }
        case 4:{
            std::cout << "Enter <side> <q0> <q1> <q2> <q3> <q4> <q5> <q6>:";
            float q0, q1, q2,q3, q4, q5, q6;
            std::cin>>inputSide>> q0 >> q1 >> q2 >> q3 >> q4 >> q5 >> q6;
            std::vector< std::vector<float> > armData;

            armSide side = inputSide == 0 ? armSide::LEFT : armSide::RIGHT;

            armData.push_back({q0, q1, q2, q3, q4, q5, q6});
            armTraj.moveArmJoints(side, armData, 2.0f);
            break;
        }

        default:{

            // Set the pose of the left arm to extend it to the front
            armTrajectory::armJointData l;
            l.side = LEFT;
            l.arm_pose = {1.57f, 1.2f, -1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
            l.time = 2;

            // Set the pose of the right arm to extend it to the front
            armTrajectory::armJointData r;
            r.side = RIGHT;
            r.arm_pose = {-1.57f, 1.2f, 1.57f, 0.0f, 0.0f, 0.0f, 0.0f};
            r.time = 2;

            // Combine the left and right arm movements
            std::vector<armTrajectory::armJointData> hug_start;
            hug_start.push_back(r);
            hug_start.push_back(l);

            // Set the pose of the left arm to embrace
            armTrajectory::armJointData l2;
            l2.side = LEFT;
            l2.arm_pose = {1.57f, 1.2f, -1.57f, -1.1f, 0.0f, 0.0f, 0.0f};
            l2.time = 2;

            // Set the pose of the right arm to embrace
            armTrajectory::armJointData r2;
            r2.side = RIGHT;
            r2.arm_pose = {-1.57f, 1.2f, 1.57f, 1.1f, 0.0f, 0.0f, 0.0f};
            r2.time = 2;

            // Combine the left and right arm movements
            std::vector<armTrajectory::armJointData> hug_end;
            hug_end.push_back(r2);
            hug_end.push_back(l2);

            // Apply the first set of arm movements
            armTraj.moveArmJoints(hug_start);
            ros::Duration(2.5).sleep();

            // Finish with the last set of arm movements
            armTraj.moveArmJoints(hug_end);

            ros::Duration(2.5).sleep();



            // Move arms to specific points in space
            geometry_msgs::Pose right;
            right.position.x = 0.8;
            right.position.y = 0.4;
            right.position.z = 1.7;
            right.orientation.w = 1.0;

            geometry_msgs::Pose left;
            left.position.x = 2.0;
            left.position.y = 2.0;
            left.position.z = 1.5;
            left.orientation.w = 1.0;

            armTrajectory::armTaskSpaceData rts;
            rts.pose = right;
            rts.time = 2.0;
            rts.side = RIGHT;

            armTrajectory::armTaskSpaceData lts;
            lts.pose = left;
            lts.time = 2.0;
            lts.side = LEFT;

            std::vector<armTrajectory::armTaskSpaceData> ts;
            ts.push_back(rts);
            ts.push_back(lts);


            armTraj.moveArmInTaskSpace(ts);
            break;
        }

        }
    }

    return 0;
}
