#include "navigation_common/map_generator.h"
#include <tough_common/val_common_names.h>


const float MapGenerator::MAP_RESOLUTION = 0.05f;
const float MapGenerator::MAP_HEIGHT = 100/MapGenerator::MAP_RESOLUTION;
const float MapGenerator::MAP_WIDTH = 100/MapGenerator::MAP_RESOLUTION;
const float MapGenerator::MAP_X_OFFSET = -50.0f;
const float MapGenerator::MAP_Y_OFFSET = -50.0f;


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
    std::fill(occGrid_.data.begin(), occGrid_.data.end(), OCCUPIED);
    visitedOccGrid_ = occGrid_;

    geometry_msgs::Pose pelvisPose;
    currentState_  = RobotStateInformer::getRobotStateInformer(nh_);
    rd_ = RobotDescription::getRobotDescription(nh_);
    currentState_->getCurrentPose(rd_->getPelvisFrame(),pelvisPose);
    ros::Duration(0.2).sleep();

    for (float x = -0.5f; x < 0.5f; x += MAP_RESOLUTION/10){
        for (float y = -0.5f; y < 0.5f; y += MAP_RESOLUTION/10){
            occGrid_.data.at(getIndex(pelvisPose.position.x + x, pelvisPose.position.y +y)) =  FREE;
            visitedOccGrid_.data.at(getIndex(pelvisPose.position.x + x, pelvisPose.position.y +y)) =  FREE;
        }
    }

//    for(float x = -0.5f; x < 1.5f; x += MAP_RESOLUTION/10 ){
//        for(float y = -0.5f; y < 0.5f; y += MAP_RESOLUTION/10 ){
//            occGrid_.data.at(getIndex(x, y)) = 0;
//            visitedOccGrid_.data.at(getIndex(x, y)) = 0;
//        }
//    }

    pointcloudSub_       = nh_.subscribe("walkway", 10, &MapGenerator::convertToOccupancyGrid, this);   // add free cells by publishing to this topic
    resetMapSub_         = nh_.subscribe("reset_map", 10, &MapGenerator::resetMap, this);
    blockMapSub_         = nh_.subscribe("/block_map", 10, &MapGenerator::updatePointsToBlock, this);   // add permanent obstacles by publishing to this topic
    clearCurrentPoseSub_ = nh_.subscribe("map/clear_current_pose", 10, &MapGenerator::clearCurrentPoseCB, this);
    mapPub_        = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 10, true);
    visitedMapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/visited_map", 10, true);
    timer_         = nh_.createTimer(ros::Duration(2), &MapGenerator::timerCallback, this);

}

MapGenerator::~MapGenerator() {
    pointcloudSub_.shutdown();
    resetMapSub_.shutdown();
    blockMapSub_.shutdown();
    timer_.stop();
}

void MapGenerator::resetMap(const std_msgs::Empty &msg) {
    std::fill(occGrid_.data.begin(), occGrid_.data.end(),  OCCUPIED);
    visitedOccGrid_ = occGrid_;


    mtx.lock();
    for (float x = -0.5f; x < 0.5f; x += MAP_RESOLUTION/10){
        for (float y = -0.5f; y < 0.5f; y += MAP_RESOLUTION/10){
            geometry_msgs::Pose pelvisPose;
            pelvisPose.position.x = x;
            pelvisPose.position.y = y;
            pelvisPose.orientation.w = 1.0f;
            currentState_->transformPose(pelvisPose, pelvisPose, rd_->getPelvisFrame(), VAL_COMMON_NAMES::WORLD_TF);
            occGrid_.data.at(getIndex(pelvisPose.position.x, pelvisPose.position.y)) =  FREE;
            visitedOccGrid_.data.at(getIndex(pelvisPose.position.x, pelvisPose.position.y)) =  FREE;
        }
    }
    mtx.unlock();
    pointsToBlock_.data.clear();
    mapPub_.publish(occGrid_);
    visitedMapPub_.publish(visitedOccGrid_);
}

void MapGenerator::clearCurrentPoseCB(const std_msgs::Empty &msg)
{
    mtx.lock();
    for (float x = -0.2f; x < 0.2f; x += MAP_RESOLUTION/10){
        for (float y = -0.2f; y < 0.2f; y += MAP_RESOLUTION/10){
            geometry_msgs::Pose pelvisPose;
            pelvisPose.position.x = x;
            pelvisPose.position.y = y;
            pelvisPose.orientation.w = 1.0f;
            currentState_->transformPose(pelvisPose, pelvisPose, rd_->getPelvisFrame(), VAL_COMMON_NAMES::WORLD_TF);
            occGrid_.data.at(getIndex(pelvisPose.position.x + x, pelvisPose.position.y +y)) =  FREE;
        }
    }
    mtx.unlock();
    mapPub_.publish(occGrid_);
}

void MapGenerator::timerCallback(const ros::TimerEvent& e){
    //update visited map
    geometry_msgs::Pose pelvisPose;
    currentState_->getCurrentPose(rd_->getPelvisFrame(),pelvisPose);

    // mark a box of 1m X 1m around robot as visited area
    mtx.lock();
    for (float x = -0.5f; x < 0.5f; x += MAP_RESOLUTION/10){
        for (float y = -0.5f; y < 0.5f; y += MAP_RESOLUTION/10){
            visitedOccGrid_.data.at(getIndex(pelvisPose.position.x + x, pelvisPose.position.y +y)) =  VISITED;
        }
    }
    mtx.unlock();

    visitedMapPub_.publish(visitedOccGrid_);
}

void MapGenerator::convertToOccupancyGrid(const sensor_msgs::PointCloud2Ptr msg) {

    if (msg->data.empty()){
        return;
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    mtx.lock();
    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y){
        float x = *iter_x;
        float y = *iter_y;

        if(occGrid_.data.at(getIndex(x,y)) ==  OCCUPIED){
            occGrid_.data.at(getIndex(x, y)) =  FREE;
        }
        //update visited map only if it is completely occupied. value = 50 means visited in that map
        if(visitedOccGrid_.data.at(getIndex(x,y)) ==  OCCUPIED){
            visitedOccGrid_.data.at(getIndex(x, y)) =  FREE;
        }
    }    
    mtx.unlock();
    mapPub_.publish(occGrid_);

}

void MapGenerator::updatePointsToBlock(const sensor_msgs::PointCloud2Ptr msg) {
    if (!occGrid_.data.empty()){
        pointsToBlock_ = *msg;
        sensor_msgs::PointCloud2Iterator<float> iter_x(pointsToBlock_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointsToBlock_, "y");

        mtx.lock();
        for(; iter_x != iter_x.end(); ++iter_x, ++iter_y){
            float x = *iter_x;
            float y = *iter_y;

            if(occGrid_.data.at(getIndex(x,y)) ==  FREE){
                occGrid_.data.at(getIndex(x, y)) =  BLOCKED;
            }
        }
        mtx.unlock();
    }
    mapPub_.publish(occGrid_);
}
