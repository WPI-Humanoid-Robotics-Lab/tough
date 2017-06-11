#include <iostream>
#include "val_common/val_common_defines.h"
#include <val_controllers/val_arm_navigation.h>


using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "val_seeds");
    ros::NodeHandle nh;

    // initializing objects
    armTrajectory armTraj(nh);

    armSide side;
    std::vector< std::vector<float> > armData;
    // seed values
    const std::vector<float> leftShoulderSeed1_={0.2,-0.2,0.2,-0.2,0.2,-0.2,0.2};
    const std::vector<float> rightShoulderSeed1_={0.2, 0.2,0.2, 0.2,0.2, 0.2,0.2};

    const std::vector<float> leftShoulderSeed2_= {-0.23,0.0,0.65,-1.49,1.29,0.0,0.0};
    const std::vector<float> rightShoulderSeed2_= {-0.23,0.0,0.65,1.49,1.29,0.0,0.0};

    const std::vector<float> leftShoulderSeed3_= {-0.81,-0.60,1.12,-1.16,1.91,0.0,0.0};
    const std::vector<float> rightShoulderSeed3_= {-0.81,0.60,1.12,1.16,1.91,0.0,0.0};

    const std::vector<float> leftShoulderSeed4_= {-0.57, -1.09, 0.65, -1.1, 1.18, 0.19, 0.31};
    const std::vector<float> rightShoulderSeed4_= {-0.57, 1.09, 0.65, 1.1, 1.18, -0.19, 0.31};

    const std::vector<float> leftShoulderSeed5_= {-0.57, -1.09, 0.65, -1.1, 0.30, 0.19, 0.31};
    const std::vector<float> rightShoulderSeed5_= {-0.57, 1.09, 0.65, 1.1, 0.30, -0.19, 0.31};

    const std::vector<float> leftShoulderSeed6_= {-0.81,-1.11,0.65,-1.14,-0.26,0.19,0.30};
    const std::vector<float> rightShoulderSeed6_= {-0.81,1.11,0.65,1.14,-0.26,-0.19,0.30};


    const std::vector<float> leftShoulderSeed7_= {-1.35,-0.76,1.70,-1.07,1.44,-0.62,0.06};
    const std::vector<float> rightShoulderSeed7_= {-1.35,0.76,1.70,1.07,1.44,0.62,0.06};

    const std::vector<float> leftShoulderSeed8_= {-1.56, -0.97, 2.18, -0.63, 0.86, -0.62, 0.15};
    const std::vector<float> rightShoulderSeed8_= {-1.56, 0.97, 2.18, 0.63, 0.86, 0.62, 0.15};


    const std::vector<float> leftShoulderSeed9_= {-0.23, -0.07, 0.75, -1.53, 1.21, -0.40, 0.0};
    const std::vector<float> rightShoulderSeed9_= {-0.23, 0.07, 0.75, 1.53, 1.21, 0.40, 0.0};


    const std::vector<float> leftShoulderSeed10_= {-0.04, -0.24, 0.49, -1.30, 0.71, 0.61, -0.24};
    const std::vector<float> rightShoulderSeed10_= {-0.04, 0.24, 0.49, 1.30, 0.71, -0.61, -0.24};

    const std::vector<float> leftShoulderSeed11_= {-0.81, 0.15, 1.65, -1.42, 1.28, 0.0, -0.23};
    const std::vector<float> rightShoulderSeed11_= {-0.81, -0.15, 1.65, 1.42, 1.28, 0.0, -0.23};


    const std::vector<float> leftShoulderSeed12_= {-1.06 ,-0.77, 0.70, -1.12 ,1.96, 0.0, 0.0};
    const std::vector<float> rightShoulderSeed12_= {-1.06 ,0.77, 0.70, 1.12 ,1.96, 0.0, 0.0};


    const std::vector<float> leftShoulderSeed13_= {-0.86,0.20,1.02,-1.67,1.40,0.0,0.0};
    const std::vector<float> rightShoulderSeed13_= {-0.86,-0.20,1.02,1.67,1.40,0.0,0.0};

    const std::vector<float> leftShoulderSeed14_= {-1.59, -0.55, 1.18, -1.05, 1.52, -0.01, 0.0};
    const std::vector<float> rightShoulderSeed14_= {-1.59, 0.55, 1.18, 1.05, 1.52, 0.01, 0.0};

    const std::vector<float> leftShoulderSeed15_= {-0.81,-0.19,0.65,-1.49,1.29,0.0,0.0};
    const std::vector<float> rightShoulderSeed15_= {-0.81,0.19,0.65,1.49,1.29,0.0,0.0};


    const std::vector<float> leftShoulderSeed16_= {-1.70, -1.04, 1.39, -1.85, 1.50, 0, 0};
    const std::vector<float> rightShoulderSeed16_= {-1.70, 1.04, 1.39, 1.85, 1.50, 0, 0};


    const std::vector<float> leftShoulderSeed17_= {-1.5, -1.04, 1.6, -1.85, -1.50, 0, 0};
    const std::vector<float> rightShoulderSeed17_= {-1.5, 1.04, 1.6, 1.85, -1.50, 0, 0};


    const std::vector<float> leftShoulderSeed18_= {0.21, -1.16, 0.0, -1.07, 1.52, 0, 0};
    const std::vector<float> rightShoulderSeed18_= {0.21, 1.16, 0.0, 1.07, 1.52, 0, 0};


    const std::vector<float> leftShoulderSeed19_= {-1.5, -1.4, 1.39, -0.9, 1.10, 0.5, 0};
    const std::vector<float> rightShoulderSeed19_= {-1.5, 1.4, 1.39, 0.9, 1.10, -0.5, 0};


    const std::vector<float> leftShoulderSeed20_= {-1.2, -1.4, 1.39, -0.9, 1.10, 0.5, 0.4};
    const std::vector<float> rightShoulderSeed20_= {-1.2, 1.4, 1.39,  0.9, 1.10, -0.5, 0.4};


    const std::vector<float> leftShoulderSeed21_= {-1.5, -1.4, 1.39, -0.9, -1.10, -0.5, 0};
    const std::vector<float> rightShoulderSeed21_= {-1.5, 1.4, 1.39, 0.9, -1.10,  0.5, 0};


    const std::vector<float> leftShoulderSeed22_= {-0.66, -1.4, 1.2, -1.49, 1.29, 0, 0.26};
    const std::vector<float> rightShoulderSeed22_= {-0.66, 1.4, 1.2, 1.49, 1.29, 0, 0.26};


    const std::vector<float> leftShoulderSeed23_= {-0.66, -1.4, 0.75, -1.49, 1.29, 0, 0.26};
    const std::vector<float> rightShoulderSeed23_= {-0.66, 1.4, 0.75, 1.49, 1.29, 0, 0.26};


    if(argc == 3)
    {
        armData.clear();
        side = std::stoi(argv[1]) == 1? armSide::RIGHT :armSide::LEFT;
        if(std::stoi(argv[1]) == 0)
        {
            if(std::stoi(argv[2]) == 1) armData.push_back(leftShoulderSeed1_);
            if(std::stoi(argv[2]) == 2) armData.push_back(leftShoulderSeed2_);
            if(std::stoi(argv[2]) == 3) armData.push_back(leftShoulderSeed3_);
            if(std::stoi(argv[2]) == 4) armData.push_back(leftShoulderSeed4_);
            if(std::stoi(argv[2]) == 5) armData.push_back(leftShoulderSeed5_);
            if(std::stoi(argv[2]) == 6) armData.push_back(leftShoulderSeed6_);
            if(std::stoi(argv[2]) == 7) armData.push_back(leftShoulderSeed7_);
            if(std::stoi(argv[2]) == 8) armData.push_back(leftShoulderSeed8_);
            if(std::stoi(argv[2]) == 9) armData.push_back(leftShoulderSeed9_);
            if(std::stoi(argv[2]) == 10) armData.push_back(leftShoulderSeed10_);
            if(std::stoi(argv[2]) == 11) armData.push_back(leftShoulderSeed11_);
            if(std::stoi(argv[2]) == 12) armData.push_back(leftShoulderSeed12_);
            if(std::stoi(argv[2]) == 13) armData.push_back(leftShoulderSeed13_);
            if(std::stoi(argv[2]) == 14) armData.push_back(leftShoulderSeed14_);
            if(std::stoi(argv[2]) == 15) armData.push_back(leftShoulderSeed15_);
            if(std::stoi(argv[2]) == 16) armData.push_back(leftShoulderSeed16_);
            if(std::stoi(argv[2]) == 17) armData.push_back(leftShoulderSeed17_);
            if(std::stoi(argv[2]) == 18) armData.push_back(leftShoulderSeed18_);
            if(std::stoi(argv[2]) == 19) armData.push_back(leftShoulderSeed19_);
            if(std::stoi(argv[2]) == 20) armData.push_back(leftShoulderSeed20_);
            if(std::stoi(argv[2]) == 21) armData.push_back(leftShoulderSeed21_);
            if(std::stoi(argv[2]) == 22) armData.push_back(leftShoulderSeed22_);
            if(std::stoi(argv[2]) == 23) armData.push_back(leftShoulderSeed23_);
        }
        else
        {
            if(std::stoi(argv[2]) == 1) armData.push_back(rightShoulderSeed1_);
            if(std::stoi(argv[2]) == 2) armData.push_back(rightShoulderSeed2_);
            if(std::stoi(argv[2]) == 3) armData.push_back(rightShoulderSeed3_);
            if(std::stoi(argv[2]) == 4) armData.push_back(rightShoulderSeed4_);
            if(std::stoi(argv[2]) == 5) armData.push_back(rightShoulderSeed5_);
            if(std::stoi(argv[2]) == 6) armData.push_back(rightShoulderSeed6_);
            if(std::stoi(argv[2]) == 7) armData.push_back(rightShoulderSeed7_);
            if(std::stoi(argv[2]) == 8) armData.push_back(rightShoulderSeed8_);
            if(std::stoi(argv[2]) == 9) armData.push_back(rightShoulderSeed9_);
            if(std::stoi(argv[2]) == 10) armData.push_back(rightShoulderSeed10_);
            if(std::stoi(argv[2]) == 11) armData.push_back(rightShoulderSeed11_);
            if(std::stoi(argv[2]) == 12) armData.push_back(rightShoulderSeed12_);
            if(std::stoi(argv[2]) == 13) armData.push_back(rightShoulderSeed13_);
            if(std::stoi(argv[2]) == 14) armData.push_back(rightShoulderSeed14_);
            if(std::stoi(argv[2]) == 15) armData.push_back(rightShoulderSeed15_);
            if(std::stoi(argv[2]) == 16) armData.push_back(rightShoulderSeed16_);
            if(std::stoi(argv[2]) == 17) armData.push_back(rightShoulderSeed17_);
            if(std::stoi(argv[2]) == 18) armData.push_back(rightShoulderSeed18_);
            if(std::stoi(argv[2]) == 19) armData.push_back(rightShoulderSeed19_);
            if(std::stoi(argv[2]) == 20) armData.push_back(rightShoulderSeed20_);
            if(std::stoi(argv[2]) == 21) armData.push_back(rightShoulderSeed21_);
            if(std::stoi(argv[2]) == 22) armData.push_back(rightShoulderSeed22_);
            if(std::stoi(argv[2]) == 23) armData.push_back(rightShoulderSeed23_);
        }
        armTraj.moveArmJoints(side, armData, 2.0f);
        ros::Duration(2.0f).sleep();

    }
}
