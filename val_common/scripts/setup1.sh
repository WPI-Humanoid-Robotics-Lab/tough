#!/bin/bash
## Usage: bash setup1.sh
## Author: sumanth
## Purpose: setups the worspace with gazebo7 and  prepares the system for ihmc controllers
##
## Options:
##   none
##

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
sudo apt-get install srcsim

#Environment Variables
echo "$(tput setaf 1)updating env variables for java$(tput sgr0)"
echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
echo 'export IS_GAZEBO=true' >> ~/.bashrc

echo "$(tput setaf 1)change owner ship of ihmc_ros_java_adapter$(tput sgr0)"
sudo chmod -R 777 /opt/ros/indigo/share/ihmc_ros_java_adapter

echo "$(tput setaf 1)copy ihmc ini file$(tput sgr0)"
mkdir -p ${HOME}/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini

echo "$(tput setaf 1)set real time prio for ihmc controller$(tput sgr0)"
sudo bash -c 'echo "@ros - rtprio 99" > /etc/security/limits.d/ros-rtprio.conf'
sudo groupadd ros
sudo usermod -a -G ros $USER

echo "$(tput setaf 1)rebooting the system...\n after reboot use the script setup2.sh$(tput sgr0)"
