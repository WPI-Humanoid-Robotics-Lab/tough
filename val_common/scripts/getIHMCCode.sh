#!/bin/bash
## Usage: bash getIHMCCode.sh
## Author: sumanth
## Purpose: Clones the required ihmc code for our final tasks
##
## Options:
##   none
##

#check if the workspace is already set
# setup the workspace
if [ -d $"/home/$USER/catkin_ws" ]; then  
  WORKSPACE="catkin_ws"
  echo "$(tput setaf 1)found workspace catkin_ws$(tput sgr0)"
elif [ -d $"/home/$USER/indigo_ws" ]; then  
  WORKSPACE="indigo_ws"
  echo "$(tput setaf 1)found workspace indigo_ws$(tput sgr0)"  
else
  "$(tput setaf 1)no workspace found try running setup scripts first $(tput sgr0)" 
   exit
fi

# create the folder for repos
mkdir ~/$WORKSPACE/src/ihmc_repos
cd ~/$WORKSPACE/src/ihmc_repos/

# clone the repos
git clone https://github.com/ihmcrobotics/ihmc-ros-control.git
git clone https://github.com/ihmcrobotics/ihmc_ros_core.git
cd ihmc_ros_core
git checkout 0.9.0

# modify the build.gradle and common.launch
cd ~/$WORKSPACE/src/ihmc_repos/ihmc_ros_core/ihmc_ros_java_adapter/
patch -p1 < ~/$WORKSPACE/src/space_robotics_challenge/val_common/patches/build.gradle.diff

cd ~/$WORKSPACE/src/ihmc_repos/ihmc_ros_core/
git apply ~/$WORKSPACE/src/space_robotics_challenge/val_common/patches/ihmc_ros_core.patch

# get the jdk 
sudo sh -c 'mkdir /usr/lib/jdk'

sudo sh -c 'cd /usr/lib/jdk && wget --no-check-certificate --no-cookies --header "Cookie: oraclelicense=accept-securebackup-cookie" http://download.oracle.com/otn-pub/java/jdk/8u112-b15/jdk-8u112-linux-x64.tar.gz && tar -xvzf jdk-8u112-linux-x64.tar.gz && rm -rf jdk-8u112-linux-x64.tar.gz'
#update the bashrc
echo "export JAVA_HOME=/usr/lib/jdk/jdk1.8.0_112
export IHMC_SOURCE_LOCATION=$HOME/ihmc-open-robotics-software
" >> ~/.bashrc


