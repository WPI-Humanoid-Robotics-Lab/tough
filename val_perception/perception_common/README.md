README
======

* This library acts as an interface so we can talk to the multisense driver easily. It acts as a one point change of all multisense behaviour in our end.
* It connects to the topics that perception_common describes for multisense autonatically. Using roslaunch parameters it is possible to over write it. All library and code that depend on this library dont have to specify the multisense topics unless they want to overwrite the normal behaviour.

Notes
-----
* It does not subscibe to the topics unless a request has been made. This prevents the ros node to be attached to a topic without needing the information.

**External dependencies** - OpenCV2, PCL 

**ROS dependencies**  - cv_bridge, roscpp 

**Nodes created**  - test_laser, test_image, test_organizedRGBD

***

perception_common provides the basic functionality to access data from the ros topics to the native PCL/OpenCV data type. It can be thought of as a wrapper that ensures that everyone follows the same convention to access the topics. Also ensures that quirks or problems in the way multisense sends data are dealt with in one location.

***

**usage**

for image
 check the test_image.cpp
 running the file:
 rosrun perception_common test_image default

***
**Comments**
* Multisense publishes intensity as uint32 but pcl::PointXYZI expects intensity to be a float datatype. The type casting is done appropriately in this package.
* All the classes are under the namespace drc_perception, this will prevent namespace colllsion
* I have typedef the data type the laser cloud publishes which is pcl::PointCloud<pcl::PointXYZI> as drc_perception::LaserPointCloud
perception common is also exported as a library so you can use it in your package.


