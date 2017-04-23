#include <val_task_common/frame_tracking.h>

frameTracking::frameTracking(ros::NodeHandle nh, std::string frame):
    nh_(nh), frame_(frame)
{
    motion_status_ = NOT_TRACKED;
    trackFrame(frame_);
}


frameTracking::~frameTracking(){

}

frame_track_status frameTracking::isInMotion()
{
    return motion_status_;
}

void  frameTracking::trackFrame(std::string frame)
{

    ros::Rate rate(10.0);
    tf::TransformListener listener;
    tf::StampedTransform transform, transform_prev;
    while (ros::ok())
    {
        try
        {
            listener.lookupTransform("/pelvis", frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        // comapre the tf
        std::cout << transform.getOrigin().getX() << " y " << transform.getOrigin().getY() << " z " << transform.getOrigin().getZ() << std::endl;
        std::cout << transform.getRotation().getX() << " y " << transform.getRotation().getY() << " z " << transform.getRotation().getZ() << " w " << transform.getRotation().getW() << std::endl;

        if (isTranformChanging(transform, transform_prev))
            ROS_INFO("motion");
        else
            ROS_INFO("stationary");

        // update the previous transform
        transform_prev = transform;

        rate.sleep();
    }
}

bool frameTracking::isTranformChanging (tf::StampedTransform transform_curr, tf::StampedTransform transform_prev)
{
    bool ret = false;
    tf::StampedTransform transform_diff;
    // get the diff between the transforms
    transform_diff.setOrigin(transform_curr.getOrigin()-transform_prev.getOrigin());
    transform_diff.setRotation(transform_curr.getRotation() - transform_prev.getRotation());

    std::cout << fabs((transform_curr.getOrigin()-transform_prev.getOrigin()).getX()) << " " << fabs((transform_curr.getOrigin()-transform_prev.getOrigin()).getY()) << " " << fabs((transform_curr.getOrigin()-transform_prev.getOrigin()).getZ()) << std::endl;
     std::cout << fabs((transform_curr.getRotation()-transform_prev.getRotation()).getX()) << " " << fabs((transform_curr.getRotation()-transform_prev.getRotation()).getY()) << " " << fabs((transform_curr.getRotation()-transform_prev.getRotation()).getZ()) << " " << fabs((transform_curr.getRotation()-transform_prev.getRotation()).getW()) << std::endl;

    // if the transform changes
    if (fabs((transform_curr.getOrigin()-transform_prev.getOrigin()).getX()) > POSITION_THRESHOLD ||
        fabs((transform_curr.getOrigin()-transform_prev.getOrigin()).getY()) > POSITION_THRESHOLD ||
        fabs((transform_curr.getOrigin()-transform_prev.getOrigin()).getZ()) > POSITION_THRESHOLD ||
        fabs((transform_curr.getRotation()-transform_prev.getRotation()).getX()) > ORIENTATION_THRESHOLD ||
        fabs((transform_curr.getRotation()-transform_prev.getRotation()).getY()) > ORIENTATION_THRESHOLD ||
        fabs((transform_curr.getRotation()-transform_prev.getRotation()).getZ()) > ORIENTATION_THRESHOLD ||
        fabs((transform_curr.getRotation()-transform_prev.getRotation()).getW()) > ORIENTATION_THRESHOLD)
    {
        ret = true;
    }

    return ret;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_tracking");
    ros::NodeHandle nh;

    // object for motion tracking
    frameTracking trackp(nh, "world");
    frameTracking trackl(nh, "rightFoot");
    frameTracking trackr(nh, "leftFoot");

    std::cout << trackp.isInMotion() << " " << trackl.isInMotion() << " " << trackr.isInMotion() << std::endl;
    ros::spin();

    return 0;
}
