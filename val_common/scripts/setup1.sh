#!/bin/bash
## Usage: bash setup1.sh
## Author: sumanth
## Purpose: setups the worspace with gazebo7 and  prepares the system for ihmc controllers
##
## Options:
##   none
##

#check if ros is installed
if  [ "$(which roscore)" -a "$(rosversion -d)" == 'indigo' ]; then
  echo "$(tput setaf 1)rosindigo is installed$(tput sgr0)"
else
# install ros-indigo
  echo "$(tput setaf 1)rosindigo is not installed, installing ros indigo$(tput sgr0)"
  echo "$(tput setaf 1)setup sources.list$(tput sgr0)"
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
  sudo apt-get update
  sudo apt-get install -y ros-indigo-desktop-full
  sudo rosdep init
  rosdep update
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  source /opt/ros/indigo/setup.bash

  echo "$(tput setaf 1)setting up workspace$(tput sgr0)"
  mkdir -p ~/indigo_ws/src && cd ~/indigo_ws/src
  catkin_init_workspace
  cp /opt/ros/indigo/share/catkin/cmake/toplevel.cmake .
  mv toplevel.cmake CMakeLists.txt
  cd ~/indigo_ws
  catkin_make
fi

#Gazebo 7 and SRCSim Installation
echo "$(tput setaf 1)Removing existing Gazebo and installing gazebo7$(tput sgr0)"
sudo rm /etc/apt/sources.list.d/gazebo*
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" > /etc/apt/sources.list.d/src-latest.list'
wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
wget -O - https://bintray.com/user/downloadSubjectPublicKey?username=bintray | sudo apt-key add -
echo "$(tput setaf 1)installing srcsim$(tput sgr0)"
sudo apt-get update
sudo apt-get install -y srcsim

#jdk 8
echo "$(tput setaf 1)Installing jdk 8$(tput sgr0)"
# Installing Oracle JDK
sudo apt-add-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install oracle-java8-installer

#OpenJDK - doesn't have javafx
#sudo add-apt-repository ppa:openjdk-r/ppa -y
#sudo apt-get update
#sudo apt-get install -y openjdk-8-jdk

echo "$(tput setaf 1)change owner ship of ihmc_ros_java_adapter$(tput sgr0)"
sudo chmod -R 777 /opt/ros/indigo/share/ihmc_ros_java_adapter

echo "$(tput setaf 1)copy ihmc ini file$(tput sgr0)"
mkdir -p ${HOME}/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini

echo "$(tput setaf 1)set real time prio for ihmc controller$(tput sgr0)"
sudo bash -c 'echo "@ros - rtprio 99" > /etc/security/limits.d/ros-rtprio.conf'
sudo groupadd ros
sudo usermod -a -G ros $USER

echo "$(tput setaf 1)rebooting the system...\n after reboot use the script setup2.sh$(tput sgr0)"
sudo reboot
