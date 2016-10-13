/**
 ********************************************************************************************************
 * @file    MultisenseImage.cpp
 * @brief   MultisenseImage class
 * @details Used to get the camera images from the camera
 ********************************************************************************************************
 */

/**
 * TODO: the camera config must be loaded from the topics for gazebo simulation also 
 */

#include <perception_common/MultisenseImage.h>
#include <perception_common/WRECS_Names.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


namespace drc_perception {

using namespace std;

/**
 *  These variables are needed to prevent multiple subscribers from being generated and to be only
 *  generated in request.
 */
bool MultisenseImage::image_callback_active_=false;
bool MultisenseImage::disp_callback_active_=false;
bool MultisenseImage::config_callback_active_=false;
bool MultisenseImage::depth_callback_active_=false;
bool MultisenseImage::cost_callback_active_=false;
/**
 *  @note do we need the NodeHandle passed??? Think benny
 */
MultisenseImage::MultisenseImage(ros::NodeHandle &n):nh_(n),
													 new_image_(false),
													 new_disp_(false),
													 new_depth_(false),
													 new_cost_(false),
													 it_(nh_),
													 sync_(nullptr )
{

	ros::NodeHandle pnh("~");

    if(!pnh.getParam("image_topic",image_topic_))
    {
    	image_topic_ = WRECS_NAMES::MULTISENSE_LEFT_IMAGE_COLOR_TOPIC;
    }
    if(!pnh.getParam("disp_topic",disp_topic_))
    {
        disp_topic_ = WRECS_NAMES::MULTISENSE_LEFT_DISPARITY_TOPIC;
    }
    if(!pnh.getParam("multisense_config_topic",multisense_topic_))
    {
    	multisense_topic_ = WRECS_NAMES::MULTISENSE_RAW_CAM_CONFIG_TOPIC;
    }
    if(!pnh.getParam("depth_topic",depth_topic_))
    {
    	depth_topic_ = WRECS_NAMES::MULTISENSE_LEFT_DEPTH_TOPIC;
    }
    if(!pnh.getParam("cost_topic",depth_cost_topic_))
    {
    	depth_cost_topic_ = WRECS_NAMES::MULTISENSE_DEPTH_COST_TOPIC;
    }
    //hard coded values specific to simulation, later on can be made to be not hard coded.
#ifdef GAZEBO_SIMULATION
    std::cout<<"Using DRCSIM dummy camera configs"<<std::endl;
    settings.Q_matrix_=cv::Mat_<double>(4,4,0.0);
    settings.Q_matrix_(0,0) =  610.1799470098168 * -0.07;
    settings.Q_matrix_(1,1) =  610.1799470098168 * -0.07;
    settings.Q_matrix_(0,3) = -610.1799470098168 * 512.5 * -0.07;
    settings.Q_matrix_(1,3) = -610.1799470098168 * 272.5 * -0.07;
    settings.Q_matrix_(2,3) =  610.1799470098168 * 610.1799470098168 * -0.07;
    settings.Q_matrix_(3,2) = -610.1799470098168;
    settings.Q_matrix_(3,3) =  0.0f;

    settings.camera_=(cv::Mat_<float>(3,3)<< 610.1799470098168	,0.0		,512.5,
    								0.0			,610.1799470098168	,272.5,
    								0.0			,0.0		,1.0);

    settings.width_=1024;
    settings.height_=544;
    settings.fps_=30.0;
    settings.exposure_=3233;
    settings.gain_=0;
    settings.baselength_=0.0700324326754;

 #endif

}

void MultisenseImage::setDepthTopic(const std::string &topic)
{
	depth_topic_ = topic;
}
/**
 * @note this callback is needed both for the instrinsic and the Q matrix
 *        it will change when the resolution changes
 */
void MultisenseImage::loadCameraConfig(const multisense_ros::RawCamConfigConstPtr &config)
{
	settings.Q_matrix_=cv::Mat_<double>(4,4,0.0);
    settings.Q_matrix_(0,0) =  config->fy * config->tx;
    settings.Q_matrix_(1,1) =  config->fx * config->tx;
    settings.Q_matrix_(0,3) = -config->fy * config->cx * config->tx;
    settings.Q_matrix_(1,3) = -config->fx * config->cy * config->tx;
    settings.Q_matrix_(2,3) =  config->fx * config->fy * config->tx;
    settings.Q_matrix_(3,2) = -config->fy;
    settings.Q_matrix_(3,3) =  0.0f;

	settings.camera_=(cv::Mat_<float>(3,3)<< config->fx	,0.0		,config->cx,
									0.0			,config->fy	,config->cy,
									0.0			,0.0		,1.0);

	settings.width_=config->width;
	settings.height_=config->height;
	settings.fps_=config->frames_per_second;
	settings.exposure_=config->exposure_time;
	settings.gain_=config->gain;
	settings.baselength_=fabs(config->tx);

	ROS_INFO_STREAM_ONCE("Received new config: "<<settings.Q_matrix_);
}
#ifndef GAZEBO_SIMULATION
void MultisenseImage::syncCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &dimg)
{

	loadImage(img);
	loadDisparityImage(dimg);
}
#else
void MultisenseImage::syncCallback(const sensor_msgs::ImageConstPtr &img, const stereo_msgs::DisparityImageConstPtr &dimg)
{

	loadImage(img);
	loadDisparityImage(dimg);
}
#endif

void MultisenseImage::syncDepthCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::ImageConstPtr &dimg, const sensor_msgs::ImageConstPtr &cimg)
{

	loadImage(img);
	loadDepthImage(dimg);
	loadCostImage(cimg);
}

/**
 * @note callback for loading images
 */
void MultisenseImage::loadImage(const sensor_msgs::ImageConstPtr &img)
{

	try
	{
		int source_type = cv_bridge::getCvType(img->encoding);
		if(source_type==CV_8UC3)
		{
			cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
		}
		else
		{
			cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
		}
		image_=cv_ptr_->image.clone();
		new_image_=true;
		img_header_=cv_ptr_->header;

		ROS_INFO_ONCE("Received new image size: %d x %d",image_.rows,image_.cols);
	 }
	 catch (cv_bridge::Exception& e)
	 {
		 ROS_ERROR_STREAM("Exception: " << e.what());
	 }

}

/**
 * @note callback for loading images
 */
void MultisenseImage::loadDepthImage(const sensor_msgs::ImageConstPtr &img)
{

	try
	{
		int source_type = cv_bridge::getCvType(img->encoding);
		if(source_type==CV_32FC1)
		{
			cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
		}
		else
		{
			ROS_ERROR_STREAM("Unsupported depth map?");
			return;
		}
		depth_=cv_ptr_->image.clone();
		new_depth_=true;
		depth_header_=cv_ptr_->header;

		ROS_INFO_ONCE("Received new depth image size: %d x %d",depth_.rows,depth_.cols);
	 }
	 catch (cv_bridge::Exception& e)
	 {
		 ROS_ERROR_STREAM("Exception: " << e.what());
	 }

}

void MultisenseImage::loadCostImage(const sensor_msgs::ImageConstPtr &img)
{

	try
	{
		int source_type = cv_bridge::getCvType(img->encoding);
		if(source_type==CV_8U)
		{
			cv_ptr_ = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
		}
		else
		{
			ROS_ERROR_STREAM("Unsupported cost map?");
			return;
		}
		cost_=cv_ptr_->image.clone();
		new_cost_=true;
		cost_header_=cv_ptr_->header;

		ROS_INFO_ONCE("Received new cost image size: %d x %d",cost_.rows,cost_.cols);
	 }
	 catch (cv_bridge::Exception& e)
	 {
		 ROS_ERROR_STREAM("Exception: " << e.what());
	 }

}

#ifdef GAZEBO_SIMULATION
/*
 *  @note callback for simulation disparity as it is published as stereo_msgs instead of sensor_msgs
 */
void MultisenseImage::loadDisparityImage(const stereo_msgs::DisparityImageConstPtr &img)
{

    try
    {

    	int source_type = cv_bridge::getCvType(img->image.encoding);
    	uint8_t depth=sensor_msgs::image_encodings::bitDepth(img->image.encoding);  //the size of the disparity data can be 16 or 32
    	if (depth == 32)
    	{
    		cv::Mat_<float> disparity(img->image.height, img->image.width,
    				                   const_cast<float*>(reinterpret_cast<const float*>(&img->image.data[0])));

    		disparity_=disparity.clone();
    		new_disp_ = true;
    		disp_header_=img->image.header;
    	}
    	else if(depth == 16)
    	{
    		cv::Mat_<uint16_t> disparityOrigP(img->image.height, img->image.width,
    		                                  const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&img->image.data[0])));
    		cv::Mat_<float>   disparity(img->image.height, img->image.width);
    		disparity = disparityOrigP / 16.0f;
    		disparity_=disparity;
    		new_disp_ = true;
    		disp_header_=img->image.header;
    	}
    	else
    		ROS_WARN("disparity depth not recognized");

    	ROS_INFO_ONCE("Received new disparity image size: %d x %d",disparity_.rows,disparity_.cols);

    }
    catch (std::exception ex)
    {    	
        ROS_ERROR_STREAM("Exception: " << ex.what());
    }

}
#endif
/**
 * @note callback for loading disparity
 */
#ifndef GAZEBO_SIMULATION
void MultisenseImage::loadDisparityImage(const sensor_msgs::ImageConstPtr &img)
{
    try
    {
    	int source_type = cv_bridge::getCvType(img->encoding);
    	uint8_t depth=sensor_msgs::image_encodings::bitDepth(img->encoding);  //the size of the disparity data can be 16 or 32
    	if (depth == 32)
    	{
    		cv::Mat_<float> disparity(img->height, img->width,
    				                   const_cast<float*>(reinterpret_cast<const float*>(&img->data[0])));

    		disparity_=disparity.clone();
    		new_disp_ = true;
    		disp_header_=img->header;
    	}
    	else if(depth == 16)
    	{
    		cv::Mat_<uint16_t> disparityOrigP(img->height, img->width,
    		                                  const_cast<uint16_t*>(reinterpret_cast<const uint16_t*>(&img->data[0])));
    		cv::Mat_<float>   disparity(img->height, img->width);
    		disparity = disparityOrigP / 16.0f;
    		disparity_=disparity.clone();
    		new_disp_ = true;
    		disp_header_=img->header;
    	}
    	else
    		ROS_WARN("disparity depth not recognized");

    	ROS_INFO_ONCE("Received new disparity image size: %d x %d , depth: %d",disparity_.rows,disparity_.cols,depth);

    }
    catch (std::exception ex)
    {
        ROS_ERROR_STREAM("Exception: " << ex.what());
    }

}
#endif




/**
 * @note starts the subscriber if not started
 */
bool MultisenseImage::giveDisparityImage(cv::Mat &disp_img)
{
	if(!disp_callback_active_)
	{
#ifndef GAZEBO_SIMULATION
		 disp_sub_ = it_.subscribe(disp_topic_, 1, &MultisenseImage::loadDisparityImage, this);
#else
		 disp_sub_ = nh_.subscribe(disp_topic_, 1, &MultisenseImage::loadDisparityImage, this);
#endif
		 disp_callback_active_=true;
		 ROS_INFO_STREAM("Listening to: "<<disp_sub_.getTopic()<<endl);
		 ros::Duration(1).sleep();
		 ros::spinOnce();
	}
    if (new_disp_)
    {
        if(disparity_.empty())
        	return false;
        disp_img = disparity_;
        new_disp_=false;
        return true;
    }

    return false;
}

bool MultisenseImage::giveDepthImage(cv::Mat &depth_img)
{
	if(!depth_callback_active_)
	{
		 depth_sub_ = it_.subscribe(depth_topic_, 1, &MultisenseImage::loadDepthImage, this);
		 depth_callback_active_=true;
		 ROS_INFO_STREAM("Listening to: "<<depth_sub_.getTopic()<<endl);
		 ros::Duration(1).sleep();
		 ros::spinOnce();
	}
    if (new_depth_)
    {
        if(depth_.empty())
        	return false;
        depth_img = depth_;
        new_depth_=false;
        return true;
    }

    return false;
}
/**
 * @note starts subscriber if not srtaed. TODO: try breaking it using multiple objects of the class
 */
bool MultisenseImage::giveImage(cv::Mat &img)
{
	if(!image_callback_active_)
	{
		cam_sub_ =it_.subscribe(image_topic_, 1, &MultisenseImage::loadImage, this);
		image_callback_active_=true;
		ROS_INFO_STREAM("Listening to: "<<cam_sub_.getTopic()<<endl);
		ros::Duration(1).sleep();
		ros::spinOnce();
	}

	if(new_image_)
	{
		img=image_;
		if(img.empty())
			return false;
		new_image_=false;
		return true;
	}
	return false;
}

bool MultisenseImage::giveCostImage(cv::Mat &img)
{
#ifdef GAZEBO_SIMULATION
	return false;
#endif
	if(!cost_callback_active_)
	{
		cost_sub_ =it_.subscribe(depth_cost_topic_, 1, &MultisenseImage::loadCostImage, this);
		cost_callback_active_=true;
		ROS_INFO_STREAM("Listening to: "<<cost_sub_.getTopic()<<endl);
		ros::Duration(1).sleep();
		ros::spinOnce();
	}

	if(new_cost_)
	{
		img=cost_;
		if(img.empty())
			return false;
		new_cost_=false;
		return true;
	}
	return false;
}

/**
 * @note none
 */
bool MultisenseImage::giveCameraInfo(cv::Mat &cam)
{
	if(!config_callback_active_)
	{
		 multisense_sub_=nh_.subscribe(multisense_topic_,1,&MultisenseImage::loadCameraConfig, this);
		 config_callback_active_=true;
		 ROS_INFO_STREAM("Listening to: "<<multisense_sub_.getTopic()<<endl);
		 ros::Duration(1).sleep();
		 ros::spinOnce();
	}
	if(!settings.camera_.empty())
	{
		cam=settings.camera_;
		return true;
	}
	return false;
}
/**
 * @note none
 */
bool MultisenseImage::giveQMatrix(cv::Mat &Q)
{
	if(!config_callback_active_)
	{
		 multisense_sub_=nh_.subscribe(multisense_topic_,1,&MultisenseImage::loadCameraConfig, this);
		 config_callback_active_=true;
		 ROS_INFO_STREAM("Listening to: "<<multisense_sub_.getTopic()<<endl);
		 ros::Duration(1).sleep();
		 ros::spinOnce();
	}
	if(!settings.Q_matrix_.empty())
	{
		Q=settings.Q_matrix_;
		return true;
	}
	return false;
}

bool MultisenseImage::giveSyncImages(cv::Mat &color, cv::Mat &disp)
{
	ROS_INFO_STREAM_ONCE("Requesting synchornized image");
	if(sync_==nullptr)
	{
		cam_sub_.shutdown();
		sync_cam_sub_=new image_transport::SubscriberFilter(it_,image_topic_, 1);
		//cam_sub_=sync_cam_sub_.getSubscriber();
		image_callback_active_=true;
		ROS_INFO_STREAM("Listening to: "<<sync_cam_sub_->getTopic()<<endl);
		disp_sub_.shutdown();
#ifndef GAZEBO_SIMULATION
        ROS_INFO("DRCSIM not enabled");
		sync_disp_sub_=new image_transport::SubscriberFilter(it_,disp_topic_, 1);
#else
		ROS_INFO_STREAM("DRCSIM topics enabled");
		sync_disp_sub_=new message_filters::Subscriber<stereo_msgs::DisparityImage>(nh_,disp_topic_, 1);
#endif
		//disp_sub_=sync_disp_sub_.getSubscriber();
		disp_callback_active_=true;
		ROS_INFO_STREAM("Listening to: "<<sync_disp_sub_->getTopic()<<endl);
#ifndef GAZEBO_SIMULATION
		sync_.reset( new message_filters::Synchronizer< exactTimePolicy >( exactTimePolicy( 1000 ), *sync_cam_sub_, *sync_disp_sub_ ));
#else
		sync_.reset( new message_filters::Synchronizer< approxTimePolicy >( approxTimePolicy( 100 ), *sync_cam_sub_, *sync_disp_sub_ ));
#endif
		sync_->registerCallback( boost::bind( &MultisenseImage::syncCallback, this, _1, _2 ) );
		 ros::Duration(1).sleep();
		 ros::spinOnce();

	}

    ROS_INFO_STREAM_ONCE("image topic: " << image_topic_);
    ROS_INFO_STREAM_ONCE("disp topic: " << disp_topic_);

	if(!giveImage(color)||!giveDisparityImage(disp))
	{
		return false;
	}
	return true;
}

bool MultisenseImage::giveSyncDepthImages(cv::Mat &color, cv::Mat &disp, cv::Mat &cost)
{
	ROS_INFO_STREAM_ONCE("Requesting synchornized depth image");
	if(sync_depth_==nullptr)
	{
		cam_sub_.shutdown();
		sync_cam_sub_=new image_transport::SubscriberFilter(it_,image_topic_, 1);
		//cam_sub_=sync_cam_sub_.getSubscriber();
		image_callback_active_=true;
		ROS_INFO_STREAM("Listening to: "<<sync_cam_sub_->getTopic()<<endl);
		depth_sub_.shutdown();
#ifndef GAZEBO_SIMULATION
        ROS_INFO("DRCSIM not enabled");
        sync_cam_depth_sub_=new image_transport::SubscriberFilter(it_,depth_topic_, 1);
        sync_cam_cost_sub_=new image_transport::SubscriberFilter(it_,depth_cost_topic_, 1);
#else
		ROS_ERROR_STREAM("Depth Image not available in simulation");
		return false;
#endif


		ROS_INFO_STREAM("Listening to: "<<sync_cam_depth_sub_->getTopic()<<endl);
		ROS_INFO_STREAM("Listening to: "<<sync_cam_cost_sub_->getTopic()<<endl);
		sync_depth_.reset( new message_filters::Synchronizer< depthImageCostExactTimePolicy >( depthImageCostExactTimePolicy( 1000 ), *sync_cam_sub_, *sync_cam_depth_sub_, *sync_cam_cost_sub_ ));
		sync_depth_->registerCallback( boost::bind( &MultisenseImage::syncDepthCallback, this, _1, _2, _3 ) );
		 ros::Duration(1).sleep();
		 ros::spinOnce();

	}


	if(!giveImage(color)||!giveDepthImage(disp)||!giveCostImage(cost))
	{
		return false;
	}
	return true;
}

//this function is a temp function that I am implementing, maybe the statistics on the fps must be maintained internally and
//not by the called class;
bool MultisenseImage::giveTime(ros::Time &time)
{
	if(disp_header_.stamp==ros::Time(0))
		time=img_header_.stamp;
	else
		time=disp_header_.stamp;
	return true;
}
bool MultisenseImage::giveSyncImageswTime(cv::Mat &color, cv::Mat &disp, ros::Time &time)
{
	if(!giveSyncImages(color,disp))
		return false;
	//assert(color.empty());
	time=img_header_.stamp;
	return true;
}
bool MultisenseImage::giveSyncDepthImageswTime(cv::Mat &color, cv::Mat &disp, cv::Mat &cost, ros::Time &time)
{
	if(!giveSyncDepthImages(color,disp,cost))
	{
		return false;
	}
	time=img_header_.stamp;
	return true;
}
/**
 * @note none
 */
MultisenseImage::~MultisenseImage() {
//	sync_->~Synchronizer();
	//sync_->~Synchronizer();
	//delete sync_;
//	sync_cam_sub_->unsubscribe();
//	sync_disp_sub_->unsubscribe();
//	delete sync_cam_sub_;
//	delete sync_disp_sub_;
}

} /* namespace drc_perception */
