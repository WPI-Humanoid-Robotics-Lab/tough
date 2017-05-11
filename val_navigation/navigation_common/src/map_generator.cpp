#include "navigation_common/map_generator.h"
#include <val_common/val_common_names.h>

size_t MapGenerator::getIndex(float x, float y){

    trimTo2DecimalPlaces(x, y);

    x -= MAP_X_OFFSET;
    y -= MAP_Y_OFFSET;

    int index_x = x/MAP_RESOLUTION;
    int index_y = y/MAP_RESOLUTION;

    size_t index = index_y*MAP_WIDTH + index_x;

    return index;
}

void MapGenerator::trimTo2DecimalPlaces(float &x, float &y) {
    int temp = round(x*(10/MAP_RESOLUTION));
    x        = floor(temp/(10.0/MAP_RESOLUTION)*100.0)/100.0;

    temp     = round(y*(10/MAP_RESOLUTION));
    y        = floor(temp/(10.0/MAP_RESOLUTION)*100.0)/100.0;

    return;
}


MapGenerator::MapGenerator(ros::NodeHandle &n):nh_(n) {
    occGrid_.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    occGrid_.info.resolution = MAP_RESOLUTION;
    occGrid_.info.height     = MAP_HEIGHT;
    occGrid_.info.width      = MAP_WIDTH;

    occGrid_.info.origin.position.x    = MAP_X_OFFSET;
    occGrid_.info.origin.position.y    = MAP_Y_OFFSET;
    occGrid_.info.origin.position.z    = 0.0;
    occGrid_.info.origin.orientation.w = 1.0;

    occGrid_.data.resize(MAP_HEIGHT*MAP_WIDTH);
    std::fill(occGrid_.data.begin(), occGrid_.data.end(), 100);
    visitedOccGrid_ = occGrid_;
    for(float x = -0.5f; x < 1.5f; x += MAP_RESOLUTION/10 ){
        for(float y = -0.5f; y < 0.5f; y += MAP_RESOLUTION/10 ){
            occGrid_.data.at(getIndex(x, y)) = 0;
            visitedOccGrid_.data.at(getIndex(x, y)) = 0;
        }
    }

    currentState_ = RobotStateInformer::getRobotStateInformer(nh_);

    pointcloudSub_ = nh_.subscribe("walkway", 10, &MapGenerator::convertToOccupancyGrid, this);
    resetMapSub_   = nh_.subscribe("reset_map", 10, &MapGenerator::resetMap, this);
    mapPub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10, true);
    ROS_INFO("Creating visited map publisher");
    visitedMapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/visited_map", 10, true);
    timer_         = nh_.createTimer(ros::Duration(2), &MapGenerator::timerCallback, this);

}

MapGenerator::~MapGenerator()
{

}

void MapGenerator::resetMap(const std_msgs::Empty &msg)
{
    std::fill(occGrid_.data.begin(), occGrid_.data.end(), 100);
    visitedOccGrid_ = occGrid_;
    for(float x = -0.5f; x < 0.5f; x += MAP_RESOLUTION/10 ){
        for(float y = -0.5f; y < 0.5f; y += MAP_RESOLUTION/10 ){
            occGrid_.data.at(getIndex(x, y)) = 0;
            visitedOccGrid_.data.at(getIndex(x, y)) = 0;
        }
    }
    mapPub_.publish(occGrid_);
    visitedMapPub_.publish(visitedOccGrid_);
}

void MapGenerator::timerCallback(const ros::TimerEvent& e){
    //update visited map
    geometry_msgs::Pose pelvisPose;
    currentState_->getCurrentPose(VAL_COMMON_NAMES::PELVIS_TF,pelvisPose);

    // mark a box of 0.5m X 0.5 m around robot as visited area
    for (float x = -0.25f; x < 0.25f; x += MAP_RESOLUTION/10){
        for (float y = -0.25f; y < 0.25f; y += MAP_RESOLUTION/10){
            visitedOccGrid_.data.at(getIndex(pelvisPose.position.x + x, pelvisPose.position.y +y)) = 50;
        }
    }

    visitedMapPub_.publish(visitedOccGrid_);
}

void MapGenerator::convertToOccupancyGrid(const sensor_msgs::PointCloud2Ptr msg) {

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");

    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y){
        float x = *iter_x;
        float y = *iter_y;
        occGrid_.data.at(getIndex(x, y)) = 0;
        //update visited map only if it is completely occupied. value = 50 means visited in that map
        if(visitedOccGrid_.data.at(getIndex(x,y)) == 100){
            visitedOccGrid_.data.at(getIndex(x, y)) = 0;
        }
    }
    mapPub_.publish(occGrid_);

}
