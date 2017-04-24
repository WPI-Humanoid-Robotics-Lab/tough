#include "navigation_common/map_generator.h"
#include <val_common/val_common_names.h>

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

    for(float x = -0.5f; x < 1.5f; x += MAP_RESOLUTION/10 ){
        for(float y = -0.7f; y < 0.7f; y += MAP_RESOLUTION/10 ){
            occGrid_.data.at(getIndex(x, y)) = 0;
        }
    }

    pointcloudSub_ = nh_.subscribe("walkway", 10, &MapGenerator::convertToOccupancyGrid, this);
    mapPub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10, true);
}

MapGenerator::~MapGenerator()
{

}

void MapGenerator::convertToOccupancyGrid(const sensor_msgs::PointCloud2Ptr msg) {

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");

    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y){
        float x = *iter_x;
        float y = *iter_y;
        occGrid_.data.at(getIndex(x, y)) = 0;
    }
    mapPub_.publish(occGrid_);

}

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



int main(int argc, char** argv) {
    ros::init(argc, argv, "map_generator");
    ros::NodeHandle n;
    MapGenerator mg(n);
    ros::spin();
    return 0;
}
