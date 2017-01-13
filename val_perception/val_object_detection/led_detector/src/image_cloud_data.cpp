#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

bool flag = true;
bool read_data = false;
std::vector<sensor_msgs::Image> image_data;
std::vector <stereo_msgs::DisparityImage> disp_data;
int maxImages = 50;
void imageCB(const sensor_msgs::Image &msg )
{

    if(flag)
    {   sensor_msgs::Image inMsg(msg);
        image_data.push_back(inMsg);
        if(image_data.size() == maxImages)
        {
            image_data.clear();
        }
        flag = !flag;
    }
}
void dispCB(const stereo_msgs::DisparityImage &msg )
{
    if(!flag)
    {
        stereo_msgs::DisparityImage inMsg(msg);
        disp_data.push_back(inMsg);
        if(disp_data.size() == maxImages)
        {
            disp_data.clear();
            read_data = true;
        }
        flag = !flag;
    }
}

int main(int argc, char **argv)
{
    ros::init (argc,argv, "image_disp_data");
    ros::NodeHandle nh;
    ros ::Subscriber image_data_sub = nh.subscribe("/multisense/camera/left/image_rect_color",0,imageCB);
    ros ::Subscriber disp_data_sub = nh.subscribe("/multisense/camera/disparity",0,dispCB);
    ros::Time begin = ros::Time::now();
    ros::Time end;
    float time_taken ;
    while(ros::ok())
    {
        if(read_data)
        {
            read_data = false;
            end  = ros::Time::now();
            time_taken = end.toSec() - begin.toSec();
            ROS_INFO("Time taken = %lf",time_taken );
            begin = ros::Time::now();
        }

        ros::spinOnce();
    }
    return 0;

}
