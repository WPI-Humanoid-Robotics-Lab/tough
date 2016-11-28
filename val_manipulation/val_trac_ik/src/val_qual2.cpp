#include <ros/ros.h>
#include <val_manipulation/val_arm_navigation_.h>
#include <val_manipulation/val_pelvis_navigation.h>

enum sm {
    WALK_TO_DOOR = 0,
    LOWER_PELVIS,
    PRESS_BUTTON,
    RETRACT_POSE_TO_WALK,
    WALK_THROUGH_DOOR
};

void executeSM(sm state);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Qual2");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    armTrajectory armtraj(nh);
    pelvisTrajectory pelvisTraj(nh);

    sm state = WALK_TO_DOOR;
    bool once = 0;


    while(ros::ok())
    {
        static int z = 0;
        //executeSM(state);

        switch (state)
        {
        case WALK_TO_DOOR:
        {
            ROS_INFO("walk to the door");
            state = LOWER_PELVIS;
            break;
        }
        case LOWER_PELVIS:
        {
            ROS_INFO("lower pelvis");

            // lower the pelvis
            pelvisTraj.controlPelvisHeight(0.87);
            state = PRESS_BUTTON;
            break;
        }
        case PRESS_BUTTON:
        {
            ROS_INFO("pressing the button");

            z++;

            if(z >20)
            {
                armtraj.buttonPressArm(RIGHT);
                z = 0;
                state = RETRACT_POSE_TO_WALK;
            }

            break;
        }
        case RETRACT_POSE_TO_WALK:
        {
            ROS_INFO("retract hand");

            z++;

            if(z >110)
            {
                armtraj.walkPoseArm(RIGHT);
                z = 0;
                state = WALK_THROUGH_DOOR;
            }

            break;
        }
        case WALK_THROUGH_DOOR:
        {
            ROS_INFO("walk through the door");

            z++;
            if(z >50)
            {
                // pelvis back to normal state
                pelvisTraj.controlPelvisHeight(1);

                // remove
                return 0;
            }
            break;
        }
        default:

            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


void executeSM(sm state)
{

}
