#!/bin/bash
## Usage: bash setup2.sh
## Author: sumanth
## Purpose: setups the system for ihmc controllers
##
## Options:
##   none
##

#check if the workspace is already set
# setup the workspace
#echo "$(tput setaf 1)Select JDK 8 in the following options$(tput sgr0)"
#sudo update-alternatives --config java
#sudo update-alternatives --config javac

echo "$(tput setaf 1)check the workspace$(tput sgr0)"
if [ -d $"/home/$USER/catkin_ws" ]; then
  WORKSPACE="catkin_ws"
  echo "$(tput setaf 1)found workspace catkin_ws$(tput sgr0)"
elif [ -d $"/home/$USER/indigo_ws" ]; then
  WORKSPACE="indigo_ws"
  echo "$(tput setaf 1)found workspace indigo_ws$(tput sgr0)"
else
  "$(tput setaf 1)no workspace found setting up indigo_ws$(tput sgr0)"
  WORKSPACE="indigo_ws"
  mkdir -p ~/indigo_ws/src && cd ~/indigo_ws/src
  catkin_init_workspace
  cd ~/indigo_ws
  catkin_make
fi


echo "$(tput setaf 1)downloading gazebo models$(tput sgr0)"
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
mkdir -p $HOME/.gazebo/models
tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
rm /tmp/default.tar.gz

#echo "$(tput setaf 1)Pre-build ihmc_ros_java_adapter$(tput sgr0)"
#source /opt/nasa/indigo/setup.bash
#roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch

echo "$(tput setaf 1)clone the whole ihmc repo$(tput sgr0)"
cd ~ && git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git

#update and source bashrc
#Environment Variables
echo "$(tput setaf 1)updating env variables for java$(tput sgr0)"

#Oracle JDK path
echo 'export JAVA_HOME=/usr/lib/jvm/java-8-oracle' >> ~/.bashrc

#OpenJDK path
#echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc

echo 'export IS_GAZEBO=true' >> ~/.bashrc
echo 'export ROBOT_NAME=VALKYRIE' >> ~/.bashrc
echo '#export ROBOT_NAME=ATLAS' >> ~/.bashrc
echo "export IHMC_SOURCE_LOCATION=$HOME/ihmc-open-robotics-software" >> ~/.bashrc
source $HOME/.bashrc

#sourcing desn't set the env variables for unknown reason. This is a workaround
export JAVA_HOME=/usr/lib/jvm/java-8-oracle
export IHMC_SOURCE_LOCATION=$HOME/ihmc-open-robotics-software

# ihmc-open-robotics-software
echo "$(tput setaf 1)compile the ihmc repo$(tput sgr0)"
cd ihmc-open-robotics-software
git stash
git pull
git checkout origin/0.9-support
./gradlew
./gradlew deployLocal

# create the folder for repos
mkdir ~/$WORKSPACE/src/ihmc_repos
cd ~/$WORKSPACE/src/ihmc_repos/

# clone the ihmc repos
echo "$(tput setaf 1)cloning the required ihmc repos$(tput sgr0)"
git clone https://github.com/ihmcrobotics/ihmc-ros-control.git
git clone https://github.com/ihmcrobotics/ihmc_ros_core.git
cd ihmc_ros_core
git checkout 0.9.2
mkdir -p ${HOME}/.ihmc
cp ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini ${HOME}/.ihmc/IHMCNetworkParameters.ini

#TODO: this to account for the fix in humanoid navigation package
#so, we clone the fixed repo
if [ -d $"/home/$USER/$WORKSPACE/src/humanoid_navigation" ]; then
  echo "$(tput setaf 1)updating the humanoid_navigation package$(tput sgr0)"
  cd ~/$WORKSPACE/src/humanoid_navigation
  git checkout indigo-devel
  git pull
  cd ~/$WORKSPACE
  catkin_make
else
  echo "$(tput setaf 1)no humanoid_navigation repo found, clonning it$(tput sgr0)"
  git clone https://github.com/ninja777/humanoid_navigation.git ~/$WORKSPACE/src/humanoid_navigation
  cd ~/$WORKSPACE/src/humanoid_navigation
  git checkout indigo-devel
  cd ~/$WORKSPACE
  catkin_make
fi

# update the src repo
if [ -d $"/home/$USER/$WORKSPACE/src/space_robotics_challenge" ]; then
  echo "$(tput setaf 1)updating the repo$(tput sgr0)"
  cd $HOME/$WORKSPACE/src/space_robotics_challenge
  git stash
  git pull
  git checkout master
  git submodule update --init --recursive
  cd $HOME/$WORKSPACE
#  catkin_make
else
  echo "$(tput setaf 1)no src repo found, clonning it$(tput sgr0)"
  git clone https://github.com/WPI-Humanoid-Research-Lab/SpaceRoboticsChallenge.git $HOME/$WORKSPACE/src/space_robotics_challenge
  cd $HOME/$WORKSPACE/src/space_robotics_challenge
  git submodule update --init --recursive
  cd $HOME/$WORKSPACE
#  catkin_make
fi

# modify the build.gradle and common.launch
# echo "$(tput setaf 1)patch the build.gradle and common.launch$(tput sgr0)"
# cd ~/$WORKSPACE/src/ihmc_repos/ihmc_ros_core/
# git apply ~/$WORKSPACE/src/space_robotics_challenge/tough_common/patches/ihmc_ros_core.patch

#echo "$(tput setaf 1)compiling the catkin_workspace$(tput sgr0)"
#cd ~/$WORKSPACE
#catkin_make

echo "$(tput setaf 1)testing the setup......$(tput sgr0)"
echo "$(tput setaf 1)launching valkyire with controllers...$(tput sgr0)"
source /opt/nasa/indigo/setup.bash
cd $HOME/$WORKSPACE
rm -rf devel/ build/
catkin_make
source devel/setup.bash

#use valkyrie_controller provided by srcsim instead of compiling it from source. 
wget -P /tmp/ http://gazebosim.org/distributions/srcsim/valkyrie_controller.tar.gz
tar -xvf /tmp/valkyrie_controller.tar.gz -C $HOME
rm /tmp/valkyrie_controller.tar.gz

# download the dependencies using gradle
roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch

echo "$(tput setaf 1)everything is setup sucessfully, launching final1.launch$(tput sgr0)"
roslaunch src_bringup final1.launch
