/**
 ********************************************************************************************************
 * @file 		ImageHelper.h
 * @brief		gives some assistive functions for MultiseseImage
 * @details 	has functions to color the disparity image.
 ********************************************************************************************************
 */
#ifndef IMAGEHELPER_H_
#define IMAGEHELPER_H_

/*** INCLUDE FILES ***/
#include <perception_common/global.h>

namespace src_perception {

class ImageHelper {
public:
	ImageHelper();
	 /**
	  * @brief   takes a disparity image and gives it color so can be visualized as an image
	  * @param   src - the disparity image that needs to be colored
	  * @param   color_image - the color image generated as a result passes as reference
	  * @param   resolution - the visualization image range
	  * @return  None
	  */
	static void colorDisparity(const cv::Mat & src, cv::Mat* &color_image, float resolution=5.0f);
	virtual ~ImageHelper();
};

} /* namespace drc_perception */

#endif /* IMAGEHELPER_H_ */
