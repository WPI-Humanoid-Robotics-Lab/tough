/**
 ********************************************************************************************************
 * @file    PointCloudHelper.cpp
 * @brief   The Point cloud helper class
 * @details gives additiional functions to work wuith the point cloud
 ********************************************************************************************************
 */

#include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisensePointCloud.h>
#include <tough_perception_common/PointCloudHelper.h>

// BW: This typedef is specific to an application and should not be here
typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

namespace tough_perception
{
// void createIntensityImage(const drc_perception::LaserPointCloud::Ptr lcloud,
//		ARCloud &cloud)
//{
//
//	cv::Mat intImg = cv::Mat::zeros(lcloud->height, lcloud->width, CV_32FC1);
//
//	ROS_INFO("width %d, height %d", intImg.cols, intImg.rows);
//	cv::Mat nImg;
//	cv::normalize(intImg, nImg, 3 * 255);
//	cv::Mat colornImg(lcloud->height, lcloud->width, CV_8UC3);
//	for (int i = 0; i < intImg.cols; i++) {
//		for (int j = 0; j < intImg.rows; j++) {
//			colornImg.at<cv::Vec3i>(j, i) = cv::Vec3i(
//					(intImg.at<float>(j, i)
//							- (intImg.at<float>(j, i)
//									- intImg.at<float>(j, i) / 255) / 255)
//							/ 255,
//					(intImg.at<float>(j, i) - intImg.at<float>(j, i) / 255)
//							/ 255, intImg.at<float>(j, i) / 255);
//		}
//	}
//	cv::imshow("intensity_image", colornImg);
//	cloud = colornImg;
//}
/**
 * @note publishes cloud like that of kinnect
 */
void PointCloudHelper::generateOrganizedRGBDCloud(const cv::Mat& dispImage, const cv::Mat& colorImage,
                                                  const cv::Mat Qmat,
                                                  tough_perception::StereoPointCloudColor::Ptr& cloud)
{
  cv::Mat xyz;
  int width = dispImage.cols;
  int height = dispImage.rows;
  cloud->resize(width * height);
  cloud->height = height;
  cloud->width = width;

  cv::reprojectImageTo3D(dispImage, xyz, Qmat, false);
  for (int u = 0; u < dispImage.rows; u++)
    for (int v = 0; v < dispImage.cols; v++)
    {
      if (dispImage.at<float>(cv::Point(v, u)) == 0.0)
        continue;
      cv::Vec3f cv_pt = xyz.at<cv::Vec3f>(cv::Point(v, u));
      tough_perception::StereoPointColor pt;
      pt.x = cv_pt.val[0];
      pt.y = cv_pt.val[1];
      pt.z = cv_pt.val[2];
      cv::Vec3b rgb = colorImage.at<cv::Vec3b>(cv::Point(v, u));
      pt.b = rgb.val[0];
      pt.g = rgb.val[1];
      pt.r = rgb.val[2];
      cloud->at(v, u) = pt;
    }
}

// void PointCloudHelper::getLaserCropToImageFOV(const cv::Size img_sz,
//						  	  	   	   	      const float laser_speed,
//						  	  	   	   	      const pcl::PointCloud<LaserPoint>::Ptr inp,
//						  	  	   	   	      pcl::PointCloud<LaserPoint>::Ptr &subCloud)
//{
//
//	if((!inp)||(inp->empty()))
//		return;
//
//	//multisense fov=80x49
//	const float hfov=80;
//	const float vfov=49;
//	float sangle=atan2(img_sz.height/2.0f,img_sz.width/2.0f)*180/M_PI;
//	int hwidth;
//	int startIdx,endIdx;
//	float theta,angle;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr laserSubCloud (new pcl::PointCloud<pcl::PointXYZ>);
//
//	for(int j=0;j<inp->height;j++)
//	{
//		theta=(360.0f/inp->height)*j;
//
//		if(theta>180)
//			theta=theta-180;
//
//		if(theta==0 || theta==-180)
//		{
//			//std::cout<<"angle : "<<theta<<std::endl;
//			hwidth=(hfov*180/(laser_speed*M_PI));
//		}
//		else if(theta<=sangle&&theta>0)
//		{
//			//std::cout<<"angle : "<<theta<<std::endl;
//			hwidth=(hfov*180/(laser_speed*M_PI))/cos(theta*M_PI/180);
//		}
//		else if((theta>sangle)&&(theta<=90))
//		{
//			angle=theta-90;
//			//std::cout<<"angle : "<<angle<<std::endl;
//			hwidth=(vfov*180/(laser_speed*M_PI))/cos(angle*M_PI/180);
//		}
//		else if((theta>90)&&(theta<180-sangle))
//		{
//			angle=theta-90;
//			//std::cout<<"angle : "<<angle<<std::endl;
//			hwidth=(vfov*180/(laser_speed*M_PI))/cos(angle*M_PI/180);
//		}
//		else
//		{
//			angle=theta-180;
//			hwidth=-(hfov*180/(laser_speed*M_PI))/cos(theta*M_PI/180);
//			//getchar();
//		}
//
//		startIdx=inp->width/2-hwidth;
//		endIdx=inp->width/2+hwidth;
////		std::cout<<theta<<" : ";
////		std::cout<<sangle<<" : ";
////		std::cout<<startIdx<<" : "<<endIdx<<std::endl;
//
//
//		auto L2norm=[](pcl::PointXYZI spt)
//					{
//					    float dx=spt.x;
//						float dy=spt.y;
//						float dz=spt.z;
//						return sqrt(dx*dx+dy*dy+dz*dz);
//					};
//		for(int i=startIdx;i<endIdx;i++)
//		{
//
//			if(L2norm(inp->at(i,j))<5.0f)
//				laserSubCloud->points.push_back(pcl::PointXYZ(inp->at(i,j).x,inp->at(i,j).y,inp->at(i,j).z));
//		}
//
//
//	}
//
//	std::cout<<"Sub cloud size: "<<laserSubCloud->points.size()<<std::endl;
//	subCloud=laserSubCloud;
//
//	//publishDebugCloud(laserSubCloud);
//
//}

void PointCloudHelper::filterLaserScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloud,
                                       std::vector<int>& indices)
{
  size_t width = laserCloud->width;
  size_t height = laserCloud->height;
  for (int j = 0; j < height; j++)
  {
    for (int l = 5; l < width - 6; l++)
    {
      int i = j * width + l;

      float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
      float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
      float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
      float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;
      if (diff > 0.05)
      {
        float depth1 =
            sqrt(laserCloud->points[i].x * laserCloud->points[i].x + laserCloud->points[i].y * laserCloud->points[i].y +
                 laserCloud->points[i].z * laserCloud->points[i].z);

        float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                            laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                            laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);
        if (depth1 > depth2)
        {
          diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
          diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
          diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;
          if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1)
          {
            indices.push_back(i - 5);
            indices.push_back(i - 4);
            indices.push_back(i - 3);
            indices.push_back(i - 2);
            indices.push_back(i - 1);
            indices.push_back(i);
          }
        }
        else
        {
          diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
          diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
          diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;
          if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
          {
            indices.push_back(i + 1);
            indices.push_back(i + 2);
            indices.push_back(i + 3);
            indices.push_back(i + 4);
            indices.push_back(i + 5);
            indices.push_back(i + 6);
          }
        }
      }
      float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
      float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
      float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
      float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;
      float dis = laserCloud->points[i].x * laserCloud->points[i].x +
                  laserCloud->points[i].y * laserCloud->points[i].y + laserCloud->points[i].z * laserCloud->points[i].z;
      if (diff > (0.25 * 0.25) / (20 * 20) * dis && diff2 > (0.25 * 0.25) / (20 * 20) * dis)
      {
        indices.push_back(i);
      }
    }
  }
}
}
