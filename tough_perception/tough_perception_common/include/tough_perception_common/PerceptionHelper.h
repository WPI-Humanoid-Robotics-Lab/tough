#ifndef PERCEPTIONHELPER_H_
#define PERCEPTIONHELPER_H_
#include <Eigen/Dense>
#include <tough_perception_common/global.h>
#include <tough_perception_common/perception_common_names.h>

namespace tough_perception
{
/**
 * @brief This function generates an Organized RGBD cloud using color and disparity images. 
 * 
 * @param dispImage disparity image
 * @param colorImage RGB image
 * @param Qmat  Q matrix for transforming pixel coordinates
 * @param cloud  output pointer to PointcloudXYZRGB 
 */
void generateOrganizedRGBDCloud(const cv::Mat &dispImage, const cv::Mat &colorImage, const Eigen::Matrix4d Qmat,
                                tough_perception::StereoPointCloudColor::Ptr &cloud);
} // namespace tough_perception

#endif