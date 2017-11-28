#!/bin/bash

terminate()
{
echo 'killed'
kill -s SIGTERM $!
exit 0
}
trap terminate SIGINT SIGTERM

while true
do
  current_time=$(date "+%Y.%m.%d-%H.%M.%S")
  data_log=$HOME'/indigo_ws/src/space_robotics_challenge/tough_common/scripts/panel_detection/footstep_'$current_time'.log'

  echo -e "\e[32mINFO:\e[0m Sourcing"
  source ~/indigo_ws/devel/setup.bash
  echo -e "\e[32mINFO:\e[0m Killing exisitng ros/gazebo instances"
  killall roslaunch gzserver rosmaster rosout
  sleep 5
  echo -e "\e[32mINFO:\e[0m Removing old log files"
  rm -rf ~/.gazebo/log/*


  roscd tough_common/scripts/panel_detection
  echo -e "\e[32mINFO:\e[0m Launching simulation"
  roslaunch tough_bringup field_bringup.launch &

  sleep 350
  roslaunch tough_footstep tough_footstep.launch --screen > $data_log &
  sleep 10

  echo -e "\e[32mINFO:\e[0m Start Gazebo Log"
  gz log -d 1
  echo -e "\e[32mINFO:\e[0m Launching panel detector"
  rosrun val_task1 panel_detection_node &

  sleep 500

  echo -e "\e[32mINFO:\e[0m Stop Gazebo Log"
  gz log -d 0
  echo -e "\e[32mINFO:\e[0m Killing qual1"
  killall roslaunch gzserver rosrun rostopic
  sleep 10

  echo -e "\e[32mINFO:\e[0m Copying log files"
  dir=`find ~/.gazebo/ -name state.log`
  state_log='state_'$current_time'.log'
  filt_log='panel_detection_'$current_time'.log'
  cp $dir $state_log
#  echo -e "\e[32mINFO:\e[0m Filtering log files"
#  gz log -e -f $state_log --filter *.pose/*.pose -z 60 > $filt_log

  roscd srcsim/worlds
  erb unique.world.erb > unique.world

done
