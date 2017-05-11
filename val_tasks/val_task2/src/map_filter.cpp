#include <val_task2/map_filter.h>

map_filter::map_filter(ros::NodeHandle& nh) : nh_(nh)
{
        //mapSub_ = nh_.subscribe("/map", 10, &map_filter::mapCB, this);
        projectedMapSub_ = nh_.subscribe("/field/projected_map", 10, &map_filter::projectedMapCB, this);

}

map_filter::~map_filter()
{
    //mapSub_.shutdown();
    projectedMapSub_.shutdown();
}

void map_filter::convertMap(nav_msgs::OccupancyGrid::Ptr msg)
{
    static bool isProcessing = false;
    if(!isProcessing)
    {
        isProcessing = true;
        ros::WallTime t = ros::WallTime::now();
        MAP_X_OFFSET = msg->info.origin.position.x;
        MAP_Y_OFFSET = msg->info.origin.position.y;
        MAP_RESOLUTION = msg->info.resolution;
        MAP_WIDTH   = msg->info.width;
        MAP_HEIGHT  = msg->info.height;

        uchar pv[msg->data.size()];
        for(size_t i = 0; i < msg->data.size(); i++)
        {
            pv[i] = (uchar) msg->data.at(i);
        }

        map_image_ = cv::Mat(cv::Size(MAP_WIDTH, MAP_HEIGHT), CV_8UC1);
        memcpy(map_image_.data, &pv, msg->data.size());
        //cv::threshold(map_image_, map_image_,50, 255, CV_THRESH_BINARY);
    }
}

void map_filter::showImage(cv::Mat image, std::string caption)
{
    cv::namedWindow( caption, cv::WINDOW_AUTOSIZE );
    cv::imshow( caption, image);
    cv::waitKey(0);
}

void map_filter::mapCB(const nav_msgs::OccupancyGrid::Ptr msg)
{
    //convertMap(msg);
    //showImage(map_image_,"map");
}

void map_filter::projectedMapCB(const nav_msgs::OccupancyGrid::Ptr msg)
{
    convertMap(msg);
    //ROS_INFO_STREAM(map_image_);
    showImage(map_image_,"projected_map");
}
