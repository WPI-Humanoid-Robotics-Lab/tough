#include <val_controllers/val_arm_navigation.h>
#include <stdlib.h>
#include <stdio.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_arm_navigation");
    ros::NodeHandle nh;
    if (argc != 9)
        return -1;
    ROS_INFO("Moving the arms");
    armTrajectory armTraj(nh);
    std::vector<float> positions;
    for (int i = 0; i < 7; i ++){
        positions.push_back(std::stof(argv[i+2]));
    }
    std::vector< std::vector<float> > armData;
    armData.push_back(positions);
    armSide side = argv[1] == "0" ? armSide::LEFT : armSide::RIGHT;

    armData.push_back(positions);
    armTraj.moveArmJoints(side, armData, 2.0f);

    ros::Duration(1).sleep();

    return 0;
}
