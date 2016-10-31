#### Team - WPI - Space Robotics Challenge
<description - coming soon>

#### Prerequisites

* Ubuntu 14.04
* Ros Indigo
* Gazebo 7

##### Gazebo 7 Installation

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-gazebo7-*
```

##### Additional Packages
* ruby (if you seeing erb related error, install ruby)    

```bash
    sudo apt-get install ruby    
```
* pcl

```bash
    sudo apt-get install ros-indigo-pcl-ros ros-indigo-pcl-conversions
```

#### Setting up workspace
Create catkin workspace. If you already have one, move to the nest step    
```bash
    mkdir -p ~/indigo_ws/src && cd ~/indigo_ws/src
    catkin_init_workspace
    cd ~/indigo_ws
    catkin_make
```
Clone the git repository    
```bash
    git clone https://gitlab.com/whrl/space_robotics_challenge.git ~/indigo_ws/src/space_robotics_challenge
```
@TODO: Create a script that would download and configure the environment

After the code is downloaded, run catkin_make.  
```bash
    cd ~/indigo_ws    
    catkin_make
```

#### Packages
* val_description   
  This package contains the robot model. All the xacro, urdf, dae files of the robot can be found in this package. This code is downloaded from [IHMC's](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/develop/Valkyrie/resources/models/val_description) repo.

* val_gazebo    
  This package contains launchers for simulation in gazebo. It also hosts the world files along with additional plugins written for various tasks.
