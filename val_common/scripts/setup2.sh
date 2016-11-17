#!/bin/bash
## Usage: bash setup2.sh
## Author: sumanth
## Purpose: setups the system for ihmc controllers
##
## Options:
##   none
##

echo "$(tput setaf 1)downloading gazebo models$(tput sgr0)"
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
rm /tmp/default.tar.gz

echo "$(tput setaf 1)Pre-build ihmc_ros_java_adapter$(tput sgr0)"
source /opt/nasa/indigo/setup.bash
roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch

echo "$(tput setaf 1)clone the whole ihmc repo$(tput sgr0)"
cd ~ && git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git

echo "$(tput setaf 1)compile the ihmc repo$(tput sgr0)"
cd ihmc-open-robotics-software
git checkout master
./gradlew 
./gradlew -q deployLocal

echo "$(tput setaf 1)checking and installing and missing ros dependecies$(tput sgr0)"
# TODO:this shoould be done with rosdep
sudo apt-get install ruby ros-indigo-pcl-ros ros-indigo-pcl-conversions ros-indigo-moveit-full ros-indigo-trac-ik ros-indigo-footstep-planner ros-indigo-humanoid-localization ros-indigo-multisense-ros

echo "$(tput setaf 1)compiling the catkin_workspace$(tput sgr0)"
cd ~/indigo_ws    
catkin_make

echo "$(tput setaf 1)testing the setup......$(tput sgr0)"
echo "$(tput setaf 1)launching valkyire with controllers...$(tput sgr0)" \
source /opt/nasa/indigo/setup.bash
cd ~/indigo_ws
rm -rf devel/ build/
catkin_make
source devel/setup.bash
roslaunch val_bringup qual2.launch


