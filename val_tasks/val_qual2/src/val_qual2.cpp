#include <ros/ros.h>
#include <val_arm_navigation_.h>
#include <val_pelvis_navigation.h>
#include <val_footstep/ValkyrieWalker.h>

enum sm {
    WALK_TO_DOOR = 0,
    LOWER_PELVIS,
    PRESS_BUTTON,
    RETRACT_POSE_TO_WALK,
    RETRACT_PELVIS,
    WALK_THROUGH_DOOR,
    ALGIN_TO_DOOR,
    EXIT
};

void executeSM(sm state);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Qual2");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    armTrajectory armtraj(nh);
    pelvisTrajectory pelvisTraj(nh);
    ValkyrieWalker walk(nh, 1.1, 1.1, 0);

    sm state = WALK_TO_DOOR;

    while(ros::ok())
    {
        static int z = 0;
        //executeSM(state);

        switch (state)
        {
        case WALK_TO_DOOR:
        {
            ROS_INFO("walking to the door");
            walk.WalkNStepsForward(6,0.475,-0.03);
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
            z++;

            if(z >10)
            {
                ROS_INFO("pressing the button");
                armtraj.buttonPressArm(RIGHT);
                z = 0;
                state = RETRACT_POSE_TO_WALK;
            }

            break;
        }
        case RETRACT_POSE_TO_WALK:
        {
            z++;

            if(z >30)
            {
                ROS_INFO("retract hand");
                armtraj.walkPoseArm(RIGHT);
                z = 0;
                state = RETRACT_PELVIS;
            }

            break;
        }
        case RETRACT_PELVIS:
        {
            z++;
            if(z >15)
            {
                ROS_INFO("retract pelvis to normal height");
                // pelvis back to normal state
                pelvisTraj.controlPelvisHeight(1);
                z=0;
                state = ALGIN_TO_DOOR;
            }
            break;

        }
        case ALGIN_TO_DOOR:
        {
            z++;
            if(z >10)
            {
                ROS_INFO("algining to door");
                walk.WalkNStepsForward(1,0,0.19);
                z=0;
                state = WALK_THROUGH_DOOR;
            }

            break;
        }
        case WALK_THROUGH_DOOR:
        {
            z++;
            if(z > 45)
            {
                ROS_INFO("walk through the door");
                walk.WalkNStepsForward(4,0.475,0);
                z=0;
                state = EXIT;
            }
            break;
        }
        default:
        EXIT:

            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


//void executeSM(sm state)
//{

//}
