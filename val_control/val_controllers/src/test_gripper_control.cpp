#include <val_control/val_gripper_control.h>
#include <stdlib.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "test_gripper_control");
    ros::NodeHandle nh;
    gripperControl gripcont(nh);

    if(argc == 2 ){
        if (argv[1][0] == '1'){
            ROS_INFO("opening grippers");
            gripcont.openGripper(LEFT);
            gripcont.openGripper(RIGHT);
        }
        else {
            ROS_INFO("closing grippers");
            gripcont.closeGripper(LEFT);
            gripcont.closeGripper(RIGHT);
        }

    }
    else if(argc == 3){
        armSide side = (armSide)std::atoi(argv[1]);
        GRIPPER_STATE state = (GRIPPER_STATE)std::atoi(argv[2]);
        gripcont.controlGripper(side, state);
    }
    else {
        ROS_INFO("Usage: rosrun %s 1 \n to open grippers \n running demo", argv[0]);
        std::vector<double> leftgripClose = {1.4, -0.55, 0.0, -0.9, -1.0};
        std::vector<double> rightgripClose = {1.4, 0.55, 0.0, 0.9, 1.0};

        gripcont.controlGripper(LEFT,leftgripClose);
        gripcont.controlGripper(RIGHT,rightgripClose);
    }
    ros::spin();

    return 0;
}
