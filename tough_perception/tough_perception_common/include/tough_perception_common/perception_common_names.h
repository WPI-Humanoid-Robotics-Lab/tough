#ifndef PERCEPTION_COMMON_NAMES_H
#define PERCEPTION_COMMON_NAMES_H

#include <string>
namespace PERCEPTION_COMMON_NAMES
{
/********* MSGS *********/

// laser topics
static const std::string MULTISENSE_LASER_SCAN_TOPIC = "/multisense/lidar_scan";

// pointcloud topics
static const std::string MULTISENSE_LASER_CLOUD_TOPIC = "/multisense/points";
static const std::string MULTISENSE_LASER_CLOUD_TOPIC2 = "/multisense/points2";

// filtered pointcloud topics
static const std::string MULTISENSE_LASER_FILTERED_CLOUD_TOPIC = "filtered_cloud";
static const std::string MULTISENSE_LASER_FILTERED_CLOUD_TOPIC2 = "filtered_cloud2";

// image topics
static const std::string MULTISENSE_LEFT_CAMERA_INFO_TOPIC = "/multisense/camera/left/camera_info";
static const std::string MULTISENSE_LEFT_IMAGE_COLOR_TOPIC = "/multisense/camera/left/image_rect_color";
static const std::string MULTISENSE_LEFT_DISPARITY_TOPIC = "/multisense/camera/disparity";
// TODO: add a depth image
static const std::string MULTISENSE_LEFT_DEPTH_TOPIC = "/multisense/depth";
static const std::string MULTISENSE_DEPTH_COST_TOPIC = "/multisense/left/cost";
static const std::string MULTISENSE_STEREO_CLOUD_TOPIC = "/multisense/image_points2";
// TODO: //needed camera calibration paramters
static const std::string MULTISENSE_RAW_CAM_CONFIG_TOPIC = "/multisense/calibration/raw_cam_config";

// multisense Topics
static const std::string MULTISENSE_CONTROL_MOTOR_TOPIC = "/multisense/set_spindle_speed";
static const std::string MULTISENSE_CONTROL_FPS_TOPIC = "/multisense/set_fps";

// topics generated by us
// TODO: update the topic once assembler is done
static const std::string ASSEMBLED_LASER_CLOUD_TOPIC = "assembled_cloud2";
static const std::string ASSEMBLED_LASER_CLOUD_TOPIC_FOR_OCTOMAP = "assembled_octomap_cloud2";

// Detector topics
static const std::string ARUCO_DETECTOR_TOPIC = "/fiducial_transforms";

// services
// TODO: once we have these
//	static const std::string PLANE_DETECTION_SERVICE = "/perception/atlas_plane_point_service";
//	static const std::string CLOSEST_POINT_SERVICE = "/perception/atlas_closest_point_service";
//	static const std::string VISUAL_HAND_SERVO_SERVICE = "/perception/hand_servo_service";
//	static const std::string OPERATIONAL_MODE_SERVICE = "/atlas/operational_mode_service";
//	static const std::string WALK_ACTION_SERVER = "/walkToTargetAS";
//	static const std::string DETECTION_SERVER = "/detect_debrisAS";
}  // namespace PERCEPTION_COMMON_NAMES

#endif
