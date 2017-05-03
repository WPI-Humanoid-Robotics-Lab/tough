#include "val_task2/path_subtraction.h"
PathSubtraction::PathSubtraction(ros::NodeHandle &nh):nh_(nh){
    occGrid_rover.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    occGrid_rover.info.resolution = MAP_RESOLUTION;
    occGrid_rover.info.height     = MAP_HEIGHT;
    occGrid_rover.info.width      = MAP_WIDTH;

    occGrid_rover.info.origin.position.x    = MAP_X_OFFSET;
    occGrid_rover.info.origin.position.y    = MAP_Y_OFFSET;
    occGrid_rover.info.origin.position.z    = 0.0;
    occGrid_rover.info.origin.orientation.w = 1.0;

    occGrid_rover.data.resize(MAP_HEIGHT*MAP_WIDTH);
    std::fill(occGrid_rover.data.begin(), occGrid_rover.data.end(), 100);

    projMap_sub_ = nh_.subscribe("/field/projected_map", 10, &PathSubtraction::getProjMap, this);
    map_sub_ = nh_.subscribe("/map", 10, &PathSubtraction::getPathMap, this);
    roverMap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/rover", 10, true);
}
PathSubtraction::~PathSubtraction(){

}

size_t PathSubtraction::getIndex1(int x, int y){
    size_t index = y*MAP_WIDTH + x;

    return index;
}

size_t PathSubtraction::getIndex2(int x, int y){
    x += (int) (fabs(occGrid_proj.info.origin.position.x)-fabs(occGrid_map.info.origin.position.x))/(MAP_RESOLUTION) + 3;
    y += (int) (fabs(occGrid_proj.info.origin.position.y)-fabs(occGrid_map.info.origin.position.y))/(MAP_RESOLUTION) +16;
    size_t index = (y*occGrid_proj.info.width) + x;
    return index;
}
void PathSubtraction::getProjMap(const nav_msgs::OccupancyGrid projMap) {
    occGrid_proj = projMap;
    for(int i = 0;i< 600;i++){
        for(int j = 0;j<MAP_WIDTH;j++){
            if(occGrid_map.data.at(getIndex1(i,j))==0 && occGrid_proj.data.at(getIndex2(i, j))==100){
                occGrid_rover.data.at(getIndex1(i, j))= 0;
            }
        }
    }
    roverMap_pub_.publish(occGrid_rover);
}

void PathSubtraction::getPathMap(const nav_msgs::OccupancyGrid pathMap) {
    occGrid_map = pathMap;
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "rover_detection");
    ros::NodeHandle n;
    PathSubtraction rd(n);
    ros::spin();
    return 0;
}
