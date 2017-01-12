#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

bool flag = true;
bool read_data = false;
std::vector<sensor_msgs::Image> image_data;
std::vector <sensor_msgs::PointCloud2> cloud_data;
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
void cloudCB(const sensor_msgs::PointCloud2 &msg )
{
    if(!flag)
    {
        sensor_msgs::PointCloud2 inMsg(msg);
        cloud_data.push_back(inMsg);
        if(cloud_data.size() == maxImages)
        {
            cloud_data.clear();
            read_data = true;
        }
        flag = !flag;
    }
}

int main(int argc, char **argv)
{
    ros::init (argc,argv, "image_cloud_data");
    ros::NodeHandle nh;
    ros ::Subscriber image_data_sub = nh.subscribe("/multisense/camera/left/image_rect_color",0,imageCB);
    ros ::Subscriber cloud_data_sub = nh.subscribe("/multisense/image_points2",0,cloudCB);
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
