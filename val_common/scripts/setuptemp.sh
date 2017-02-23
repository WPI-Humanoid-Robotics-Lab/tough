
echo "$(tput setaf 1)downloading gazebo models$(tput sgr0)"
wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
tar -xvf /tmp/default.tar.gz -C $HOME/.gazebo/models --strip 1
rm /tmp/default.tar.gz

#echo "$(tput setaf 1)Pre-build ihmc_ros_java_adapter$(tput sgr0)"
#source /opt/nasa/indigo/setup.bash
#roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch

echo "$(tput setaf 1)clone the whole ihmc repo$(tput sgr0)"
cd ~ && git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git

# get the jdk 
echo "$(tput setaf 1)get the jdk$(tput sgr0)"
sudo sh -c 'mkdir /usr/lib/jdk'
sudo sh -c 'cd /usr/lib/jdk && wget --no-check-certificate --no-cookies --header "Cookie: oraclelicense=accept-securebackup-cookie" http://download.oracle.com/otn-pub/java/jdk/8u112-b15/jdk-8u112-linux-x64.tar.gz && tar -xvzf jdk-8u112-linux-x64.tar.gz && rm -rf jdk-8u112-linux-x64.tar.gz'

#update and source bashrc
echo "$(tput setaf 1)update bashrc with environment variables$(tput sgr0)"
echo "export JAVA_HOME=/usr/lib/jdk/jdk1.8.0_112
export IHMC_SOURCE_LOCATION=$HOME/ihmc-open-robotics-software
" >> ~/.bashrc
source $HOME/.bashrc

#sourcing desn't set the env variables for unknown reason. This is a workaround
export JAVA_HOME=/usr/lib/jdk/jdk1.8.0_112 
export IHMC_SOURCE_LOCATION=$HOME/ihmc-open-robotics-software

# ihmc-open-robotics-software
echo "$(tput setaf 1)compile the ihmc repo$(tput sgr0)"
cd ihmc-open-robotics-software
git stash
git pull
git checkout origin/0.9-support
./gradlew -q deployLocal

# create the folder for repos
mkdir ~/catkin_ws/src/ihmc_repos
cd ~/catkin_ws/src/ihmc_repos/

# clone the ihmc repos
echo "$(tput setaf 1)cloning the required ihmc repos$(tput sgr0)"
git clone https://github.com/ihmcrobotics/ihmc-ros-control.git
git clone https://github.com/ihmcrobotics/ihmc_ros_core.git
cd ihmc_ros_core
git checkout 0.9.0

echo "$(tput setaf 1)checking and installing and missing ros dependecies$(tput sgr0)"
# TODO:this shoould be done with rosdep
sudo apt-get install ruby ros-indigo-pcl-ros ros-indigo-pcl-conversions ros-indigo-moveit-full ros-indigo-trac-ik ros-indigo-footstep-planner ros-indigo-humanoid-localization ros-indigo-multisense-ros  ros-indigo-laser-assembler

#TODO: this to account for the fix in humanoid navigation package
#so, we clone the fixed repo
if [ -d $"~/catkin_ws/src/humanoid_navigation" ]; then  
  echo "$(tput setaf 1)updating the humanoid_navigation package$(tput sgr0)"
  cd ~/catkin_ws/src/humanoid_navigation
  git checkout indigo-devel
  git pull
  cd ~/catkin_ws    
  catkin_make
else
  echo "$(tput setaf 1)no humanoid_navigation repo found, clonning it$(tput sgr0)" 
  git clone https://github.com/ninja777/humanoid_navigation.git ~/catkin_ws/src/humanoid_navigation
  cd ~/catkin_ws/src/humanoid_navigation
  git checkout indigo-devel
  cd ~/catkin_ws   
  catkin_make
fi

# update the src repo
if [ -d $"~/catkin_ws/src/space_robotics_challenge" ]; then  
  echo "$(tput setaf 1)updating the repo$(tput sgr0)"
  cd $HOME/catkin_ws/src/space_robotics_challenge
  git stash
  git pull
  git checkout master
  cd $HOME/catkin_ws
  catkin_make
else
  echo "$(tput setaf 1)no src repo found, clonning it$(tput sgr0)" 
  git clone https://gitlab.com/whrl/space_robotics_challenge.git $HOME/catkin_ws/src/space_robotics_challenge
  cd $HOME/catkin_ws
  catkin_make
fi

# modify the build.gradle and common.launch
echo "$(tput setaf 1)patch the build.gradle and common.launch$(tput sgr0)"
cd ~/catkin_ws/src/ihmc_repos/ihmc_ros_core/
git apply ~/catkin_ws/src/space_robotics_challenge/val_common/patches/ihmc_ros_core.patch

echo "$(tput setaf 1)compiling the catkin_workspace$(tput sgr0)"
cd ~/catkin_ws     
catkin_make

echo "$(tput setaf 1)testing the setup......$(tput sgr0)"
echo "$(tput setaf 1)launching valkyire with controllers...$(tput sgr0)" 
source /opt/nasa/indigo/setup.bash
cd $HOME/catkin_ws
rm -rf devel/ build/
catkin_make
source devel/setup.bash
#echo "$(tput setaf 1)everything is setup sucessfully, launching final1.launch$(tput sgr0)"
#roslaunch val_bringup final1.launch 


