## Team - WPI - Space Robotics Challenge
This is the main repo for Team-WPI's participation in Space Robotics Challenge. 

#### Prerequisites
  * Disable hyper-threading, if it is enabled in bios
  * Ubuntu 14.04
  * Ros Indigo

#### Gazebo 7 and SRCSim Installation

```bash
    sudo rm /etc/apt/sources.list.d/gazebo*
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -

    sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" > /etc/apt/sources.list.d/src-latest.list'
    wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
    wget -O - https://bintray.com/user/downloadSubjectPublicKey?username=bintray | sudo apt-key add -

    sudo apt-get update
    sudo apt-get install srcsim
```    

#### Environment Variables

```bash
    echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
    echo 'export IS_GAZEBO=true' >> ~/.bashrc
```

#### Change ownership of ihmc_ros_java_adapter package. 
This ROS package requires to write some files in its installation directory at runtime. We're working on a fix for this issue. In the meantime, please change the ownership of this directory to your user.

`if you are the only user on the computer, use this`
```
    sudo chown -R $USER:$USER /opt/ros/indigo/share/ihmc_ros_java_adapter
```
`else, use this`
```
    sudo chmod -R 777 /opt/ros/indigo/share/ihmc_ros_java_adapter
```

#### Copy the IHMC networking `ini` file 

```
    mkdir -p ${HOME}/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini
```

#### Real-time scheduling priority
Increase real-time scheduling priority for current user (rtprio), which is required by the IHMC controller. Add current user to ros group:

```
    sudo bash -c 'echo "@ros - rtprio 99" > /etc/security/limits.d/ros-rtprio.conf'
    sudo groupadd ros
    sudo usermod -a -G ros $USER
```

## Reboot the system

#### Download Gazebo models

```
    wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
    tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
    rm /tmp/default.tar.gz
```

#### Pre-build `ihmc_ros_java_adapter`. 
Open a new terminal and run:

```
    source /opt/nasa/indigo/setup.bash
    roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch
```
## Hack to use ihmc controllers until they provide a fix

#### Clone IHMC's open robotics software repo

```
    cd ~ && git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git
```
####   Compile 
Compile the java code that you just cloned (https://xkcd.com/303)
```
    cd ihmc-open-robotics-software
    git checkout master
    ./gradlew 
    ./gradlew -q deployLocal
```

#### Setting up workspace
Create catkin workspace. If you already have one, move to the [next](#test-your-installation) step    
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

## Test your installation

#### Open a new terminal and type:

```
    source /opt/nasa/indigo/setup.bash
    cd ~/indigo_ws
    rm -rf devel/ build/
    catkin_make
    source devel/setup.bash
    roslaunch val_bringup qual2.launch
```

	
#### Additional Packages
* ruby, pcl, MoveIt!, Trac\_IK, footstep\_planner, humanoid\_localization, Multisense

```bash
    sudo apt-get install ruby ros-indigo-pcl-ros ros-indigo-pcl-conversions ros-indigo-moveit-full ros-indigo-trac-ik ros-indigo-footstep-planner ros-indigo-humanoid-localization ros-indigo-multisense-ros
```
#### Usage
* To start qual task 1 with controller

```bash
    roslaunch val_bringup qual1.launch
```
* To start qual task 1 without controller

```bash
    roslaunch val_bringup qual1.launch controller:=false
```
* To start qual task 2 with controller

```bash
    roslaunch val_bringup qual2.launch
```
* To start qual task 2 without controller

```bash
    roslaunch val_bringup qual2.launch controller:=false
```
 

#### Packages
* val_description   
  This package contains the robot model. All the xacro, urdf, dae files of the robot can be found in this package. This code is downloaded from [IHMC's](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/develop/Valkyrie/resources/models/val_description) repo.

* val_gazebo    
  This package contains launchers for simulation in gazebo. It also hosts the world files along with additional plugins written for various tasks.

* val_moveit/val_moveit_config
  This package contains moveit configuration files with "trac_ik" solver for Valkyrie generated by MoveIt! setup assistant. This can be used for motion planning tasks. The robot groups and poses can be edited by editing this package in moveit_setup_assistant.
  Refer [MoveIt! tutorials](http://docs.ros.org/indigo/api/moveit_tutorials/html/) and [Trac_ik](https://bitbucket.org/traclabs/trac_ik) for more details.
* val_moveit/val_moveit_tasks
  This package conatains tasks that can be performed using MoveIt! code API.
