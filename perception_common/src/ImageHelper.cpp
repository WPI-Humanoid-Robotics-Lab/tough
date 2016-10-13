/**
 ********************************************************************************************************
 * @file    ImageHelper.cpp
 * @brief   Image helper class
 * @details additional functions to the Multisense Images
 ********************************************************************************************************
 */

#include <perception_common/ImageHelper.h>

namespace drc_perception {
/**
 * @note none
 */
ImageHelper::ImageHelper() {
	// TODO Auto-generated constructor stub

}
/**
 * @note - BW: the color_image argument, looks like a pointer to a reference.. double pointer?
 */
void ImageHelper::colorDisparity(const cv::Mat &src, cv::Mat* &color_image, float resolution)
{
	cv::Mat *disp_img= new cv::Mat(src.size(), CV_8UC3);
	std::vector<cv::Mat> hsv_array(3);
	hsv_array[0] = cv::Mat(src.size(), CV_8UC1);
	src.convertTo(hsv_array[0],CV_8UC1, resolution, 0.);
	hsv_array[1] = cv::Mat(src.size(), CV_8UC1, 255);
	hsv_array[2] = cv::Mat(src.size(), CV_8UC1, 255);
	cv::Mat hsv(src.size(), CV_8UC3);
	cv::merge(hsv_array, hsv);
	cv::cvtColor(hsv, *disp_img, CV_HSV2RGB);
	color_image=disp_img;

}
/**
 * @note none
 */
ImageHelper::~ImageHelper() {
	// TODO Auto-generated destructor stub
}

} /* namespace drc_perception */
