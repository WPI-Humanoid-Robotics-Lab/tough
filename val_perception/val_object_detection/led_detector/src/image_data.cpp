#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

bool flag = true;
bool read = false;
std::vector<sensor_msgs::Image> image_data;
std::vector <sensor_msgs::PointCloud2> cloud_data;
void imageCB(const sensor_msgs::Image &msg )
{

    if(flag)
    {
        image_data.push_back(msg);

        if(image_data.size() == 50)
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
        cloud_data.push_back(msg);

        if(cloud_data.size() == 50)
        {
            cloud_data.clear();
            read = true;
        }
        flag = !flag;
    }


}

int main(int argc, char **argv)
{
    ros::init (argc,argv, "image_cloud_data");
    ros::NodeHandle nh;
    ros ::Subscriber image_data_sub = nh.subscribe("/multisense/camera/left/image_rect_color",10,imageCB);
    ros ::Subscriber cloud_data_sub = nh.subscribe("/multisense/image_points2",10,cloudCB);
    ros::Time begin = ros::Time::now();
    ros::Time end = ros::Time::now();
    double time_taken ;
    while(ros::ok())
    {
        if(read)
        {
            read = false;
            end  = ros::Time::now();
            time_taken = end - begin;
            ROS_INFO("Time taken = %lf",time_taken );
            begin = ros::Time::now();
        }

        ros::spinOnce();
    }
    return 0;

}
