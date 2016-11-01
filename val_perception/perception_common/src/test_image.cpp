/**
 ********************************************************************************************************
 * @file    test_image.cpp
 * @brief   tests if we are avke to get image from the multisense
 * @details Used to test if the images are being published by the Multisense
 ********************************************************************************************************
 */
#include <perception_common/MultisenseImage.h>
#include <perception_common/ImageHelper.h>
#include <perception_common/utils.h>
#include <thread>

TimePlot time_data(40);
Gnuplot *plot_;
using namespace std;

void testretriveImageWithoutSpin()
{
    ros::NodeHandle nh;
    src_perception::MultisenseImage image_assistance(nh);

    cv::Mat disp;
    image_assistance.giveImage(disp);
    int ctr=0;
    while(ros::ok())
    {
        if(ctr++>1000)
            break;
        image_assistance.giveImage(disp);
        // check if the image is not void
        if (!disp.empty())
        {
            cv::imshow("left raw", disp);
            ros::spinOnce();
            cv::waitKey(1);
        }
        else
        {
            ROS_WARN("No image published yet");
        }
    }
}

void display()
{
    while(ros::ok())
        time_data.plot(plot_);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"image_test");

    if (argc <= 1)
    {
        ROS_ERROR("usgae: <node> <param>");
        ROS_ERROR("param: ");
        ROS_ERROR("deafult: displays the left image ");
        ROS_ERROR("disparity: displays disparity image ");
        exit(0);
    }

    if (std::string(argv[1]) == "default")
    {
        ROS_INFO("default");
        testretriveImageWithoutSpin();
    }
    else if (std::string(argv[1]) == "disparity")
    {
        ros::NodeHandle nh;
        src_perception::MultisenseImage image_assistance(nh);
        cv::Mat color_image;
        cv::Mat_<float> disp;
        cv::namedWindow("Color Image");
        cv::Mat *disp_img;
        ros::Time time;
        ros::Time ptime;

        float camera_fps;
        time_data.add("device fps");
        plot_=new Gnuplot();
        time_data.setup(plot_);

        int frame_id=0;
        std::thread thread(display);


        while(ros::ok())
        {
//            if(image_assistance.giveImage(color_image))
//            {
//                image_assistance.giveTime(time);
//                cv::imshow("Color Image",color_image);
//                assert(ptime<time);
//                ros::Duration d(time-ptime);
//                camera_fps=1.0/d.toSec();
//                ptime=time;
//                time_data.updateValue(frame_id++,"device fps",camera_fps);
//                cv::waitKey(1);
//            }
            if(image_assistance.giveDisparityImage(disp))
            {
                src_perception::ImageHelper::colorDisparity(disp,disp_img);
                cv::imshow("Disparity Image",*disp_img);
                image_assistance.giveTime(time);
                assert(ptime<time);
                ros::Duration d(time-ptime);
                camera_fps=1.0/d.toSec();
                ptime=time;
                time_data.updateValue(frame_id++,"device fps",camera_fps);
                cv::waitKey(1);
            }
//            if(image_assistance.giveSyncImageswTime(color_image,disp,time))
//            {
//                cv::imshow("Color Image",color_image);
//                src_perception::ImageHelper::colorDisparity(disp,disp_img);
//                cv::imshow("Disparity Image",*disp_img);
//                cv::waitKey(1);
//                assert(ptime<time);
//                ros::Duration d(time-ptime);
//                camera_fps=1.0/d.toSec();
//                //clearing up the pointers
//                ptime=time;
//                time_data.updateValue(frame_id++,"device fps",camera_fps);
//            }
            ros::spinOnce();
        }
    }

}



