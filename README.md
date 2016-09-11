#### Team - WPI - Space Robotics Challenge
<description - coming soon>

#### Prerequisites

* Ubuntu 14.04
* Ros Indigo
* Gazebo 7


#### Setting up workspace

Clone the git repository    
```git clone https://gitlab.com/whrl/space_robotics_challenge.git ~/indigo_ws```

After the code is downloaded, run catkin_make.  
```cd ~/indigo_ws    
catkin_make```

#### Packages
* val_description   
  This package contains the robot model. All the xacro, urdf, dae files of the robot can be found in this package. This code is downloaded from [IHMC's](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/develop/Valkyrie/resources/models/val_description) repo.

* val_gazebo    
  This package contains launchers for simulation in gazebo. It also hosts the world files along with additional plugins written for various tasks.
