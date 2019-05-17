README
======

* This library acts as an interface so we can talk to the multisense driver easily. It acts as a one point change of all multisense behaviour in our end.
* It connects to the topics that tough_perception_common describes for multisense automatically. Using roslaunch parameters it is possible to over write it. All library and code that depend on this library dont have to specify the multisense topics unless they want to overwrite the normal behaviour.

Notes
-----
**External dependencies** - OpenCV2, PCL 

**ROS dependencies**  - cv_bridge, roscpp 

***

tough_perception_common provides the basic functionality to access data from the ros topics to the native PCL/OpenCV data type. It can be thought of as a wrapper that ensures that everyone follows the same convention to access the topics. Also ensures that quirks or problems in the way multisense sends data are dealt with in one location.

***

**usage**

To download rosbags captured from actual robot for testing MultisenseInterface
```bash
cd data
./download_data.sh
```

check out the `test_multisense_image` for the usage

***
**Comments**
* Multisense publishes intensity as uint32 but pcl::PointXYZI expects intensity to be a float datatype. The type casting is done appropriately in this package.
* All the classes are under the namespace src_perception, this will prevent namespace colllsion
* perception common can be exported as a library in different package.


