#include <tough_controller_interface/arm_control_interface.h>
#include <tough_controller_interface/robot_state.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_wrist");
    ros::NodeHandle nh;
    armTrajectory arm_controller(nh);
    ROS_INFO("Moving the wrist");

    RobotStateInformer* obj = RobotStateInformer::getRobotStateInformer(nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    for (int i = 0; i < 5; ++i) {
       ros::spinOnce();
       ros::Duration(0.2).sleep();
    }

    float yaw,roll,pitch;
    RobotSide side;
    int retry = 0;
    if(argc =5){
        side = std::atoi(argv[1]) == 0 ? RobotSide::LEFT : RobotSide::RIGHT;

        if(side == RobotSide::LEFT){
            std::vector<float> positions;
            while(!obj->getJointPositions("left_arm", positions)){
                ros::spinOnce();
                ros::Duration(0.2).sleep();
                ROS_INFO("Retry count : %d", retry++);
//                if(retry > 20) break;
            }
            yaw = positions[4];
            roll = positions[5];
            pitch = positions[6];
        }
        else
        {
            std::vector<float> positions;
            while(!obj->getJointPositions("right_arm", positions)){
                ros::spinOnce();
                ros::Duration(0.2).sleep();
                ROS_INFO("Retry count : %d", retry++);
//                if(retry > 20) break;
            }
            yaw = positions[4];
            roll = positions[5];
            pitch = positions[6];
        }

        std::cout<<"current yaw"<<yaw<<"\n";
        std::cout<<"current roll"<<roll<<"\n";
        std::cout<<"current pitch"<<pitch<<"\n";

        yaw+=std::atof(argv[4]);
        roll+=std::atof(argv[2]);
        pitch+=std::atof(argv[3]);

        std::cout<<"current yaw"<<yaw<<"\n";
        std::cout<<"current roll"<<roll<<"\n";
        std::cout<<"current pitch"<<pitch<<"\n";

        arm_controller.moveArmJoint(side,4,yaw);
        ros::Duration(0.2).sleep();
        arm_controller.moveArmJoint(side,5,roll);
        ros::Duration(0.2).sleep();
        arm_controller.moveArmJoint(side,6,pitch);
        ros::Duration(3).sleep();
    }
    else std::cout<<"Invalid Input \n";

    ros::spinOnce();
    ros::Duration(2).sleep();
    return 0;
}

