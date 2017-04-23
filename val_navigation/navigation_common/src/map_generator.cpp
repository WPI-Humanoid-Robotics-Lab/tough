#include "navigation_common/map_generator.h"
#include <val_common/val_common_names.h>

MapGenerator::MapGenerator(ros::NodeHandle &n):nh_(n) {
    occGrid_.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    occGrid_.info.resolution = MAP_RESOLUTION;
    occGrid_.info.height     = MAP_HEIGHT;
    occGrid_.info.width      = MAP_WIDTH;

    occGrid_.info.origin.position.x    = 0.0;
    occGrid_.info.origin.position.y    = 0.0;
    occGrid_.info.origin.position.z    = 0.0;
    occGrid_.info.origin.orientation.w = 1.0;

    occGrid_.data.resize(MAP_HEIGHT*MAP_WIDTH);
    std::fill(occGrid_.data.begin(), occGrid_.data.end(), 1);

    pointcloudSub_ = nh_.subscribe("walkway_filtered_points2", 10, &MapGenerator::convertToOccupancyGrid, this);
    mapPub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10, true);
}

void MapGenerator::convertToOccupancyGrid(const sensor_msgs::PointCloud2Ptr msg) {

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");

    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y){
        float x = *iter_x;
        float y = *iter_y;
        convertToMapIndices(x, y);
        occGrid_.data.at(x*occGrid_.info.width + y) = 0;
    }

    mapPub_.publish(occGrid_);

}

void MapGenerator::convertToMapIndices(float &x, float &y) {
    int temp = round(x*(10/occGrid_.info.resolution));
    x        = floor(temp/(10.0/occGrid_.info.resolution)*100.0)/100.0;

    temp     = round(y*(10/occGrid_.info.resolution));
    y        = floor(temp/(10.0/occGrid_.info.resolution)*100.0)/100.0;

    return;
}



int main(int argc, char** argv) {

    return 0;
}
