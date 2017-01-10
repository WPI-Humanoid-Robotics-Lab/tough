#include <ros/ros.h>
#include <val_manipulation/val_arm_navigation_.h>
#include <val_manipulation/val_pelvis_navigation.h>
#include <val_footstep/ValkyrieWalker.h>

enum sm {
    WALK_TO_DOOR = 0,
    LOWER_PELVIS,
    PREPARE_PRESS_BUTTON,
    PRESS_BUTTON,
    RETRACT_POSE_TO_WALK,
    RETRACT_PELVIS,
    WALK_THROUGH_DOOR,
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
    // play with these time later
    ValkyrieWalker walk(nh, 0.6, 0.6);

    sm state = PREPARE_PRESS_BUTTON;

    while(ros::ok())
    {
        static int z = 0;
        //executeSM(state);

        switch (state)
        {
        case PREPARE_PRESS_BUTTON:
        {
            ROS_INFO("preparing the arm to press the button");
            armtraj.buttonPressPrepareArm(RIGHT);
            walk.WalkNStepsForward(1,0.35,-0.18,false,RIGHT);
            state = WALK_TO_DOOR;

            break;
        }
        case WALK_TO_DOOR:
        {
            ROS_INFO("walking to the door");
            walk.WalkNStepsForward(5,0.51,0);
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
            armtraj.buttonPressArm(RIGHT);
            //the above trajectory takes 1 sec
            ros::Duration(1).sleep();
            state = RETRACT_POSE_TO_WALK;

            break;
        }
        case RETRACT_POSE_TO_WALK:
        {
            ROS_INFO("retract hand");
            armtraj.walkPoseArm(RIGHT);
            //the above trajectory takes 1 sec
            ros::Duration(1).sleep();
            state = RETRACT_PELVIS;

            break;
        }
        case RETRACT_PELVIS:
        {
            ROS_INFO("retract pelvis to normal height");
            pelvisTraj.controlPelvisHeight(1);
            state = WALK_THROUGH_DOOR;

            break;

        }
        case WALK_THROUGH_DOOR:
        {
            ROS_INFO("algining and walking through to door");
            // create a list of these steps
            walk.setWalkParms(0.65,0.65, 0);
            walk.WalkNStepsForward(1, 0, 0.19, true, LEFT);
            walk.WalkNStepsForward(1, 0.5, 0.19, true, RIGHT);
            walk.WalkNStepsForward(1, 1, 0, true, LEFT);
            walk.WalkNStepsForward(1, 1, 0, true, RIGHT);
            walk.WalkNStepsForward(1, 1, 0, true, LEFT);
            walk.WalkNStepsForward(1, 1, 0, false, RIGHT);
            state = EXIT;

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
