README
======

* This library acts as an interface so we can talk to the multisense driver easily. It acts as a one point change of all multisense behaviour in our end.
* It connects to the topics that Wrecs_common describes for multisense autonatically. Using roslaunch parameters it is possible to over write it. All library and code that depend on this library dont have to specify the multisense topics unless they want to overwrite the normal behaviour.

Notes
-----
* It does not subscibe to the topics unless a request has been made. This prevents the ros node to be attached to a topic without needing the information.
* This node also generates a perception_common_drcsim version of the library that compensates for the name and behaviour change in Gazebo simulation. To prevent the perception_common_drcsim libraries to be generated use GAZEBO_SIMULATION=OFF.
* The perception_common_drcsim is not exported in catkin by default. To use it you need to explictly specify it.

**External dependencies** - OpenCV2, PCL 

**ROS dependencies**  - cv_bridge, roscpp 

**Nodes created**  - test_laser test_image 

**launch file description**

* laser_topic - The topic containing the laser point cloud
* stereo_topic - the topic containing the stereo point cloud 

***

perception_common provides the basic functionality to access data from the ros topics to the native PCL/OpenCV data type. It can be thought of as a wrapper that ensures that everyone follows the same convention to access the topics. Also ensures that quirks or problems in the way multisense sends data are dealt with in one location.

***
**Samples**

[For obtaining images in cv::Mat](https://github.com/WPI-Atlas-Lab/drc/blob/master/wrecs_perception/perception_common/src/test_image.cpp)

[For obtaining laser pointcloud in pcl::PointCloud](https://github.com/WPI-Atlas-Lab/drc/blob/master/wrecs_perception/perception_common/src/test_laser.cpp)

***
**Comments**
* Multisense publishes intensity as uint32 but pcl::PointXYZI expects intensity to be a float datatype. The type casting is done appropriately in this package.
* All the classes are under the namespace drc_perception, this will prevent namespace colllsion
* I have typedef the data type the laser cloud publishes which is pcl::PointCloud<pcl::PointXYZI> as drc_perception::LaserPointCloud
perception common is also exported as a library so you can use it in your package.


