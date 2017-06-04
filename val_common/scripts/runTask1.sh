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
  cd $HOME
  if [ ! -d ~/indigo_ws/src/space_robotics_challenge/val_tasks/val_task1/log ]; then
    mkdir -p ~/indigo_ws/src/space_robotics_challenge/val_tasks/val_task1/log;
  fi

  data_log=$HOME'/indigo_ws/src/space_robotics_challenge/val_tasks/val_task1/log/whrlT1_'$current_time'.log'
  data_log2=$HOME'/indigo_ws/src/space_robotics_challenge/val_tasks/val_task1/log/task1_'$current_time'.log'

  echo -e "\e[32mINFO:\e[0m Sourcing"
  source ~/indigo_ws/devel/setup.bash
  echo -e "\e[32mINFO:\e[0m Killing exisitng ros/gazebo instances"
  killall roslaunch gzserver rosmaster rosout
  sleep 10
  echo -e "\e[32mINFO:\e[0m Removing old log files"
  rm -rf ~/.gazebo/log/*

  # roscd val_common/scripts/task2
  echo -e "\e[32mINFO:\e[0m Launching SRCSIM"
  roslaunch srcsim unique_task1.launch init:=true &

  sleep 150
  
  roslaunch val_bringup whrl.launch --screen > $data_log &
  # stdbuf -oL roslaunch val_bringup whrl_task1.launch --screen &> $data_log

#  sleep 30
  

  echo -e "\e[32mINFO:\e[0m Start Gazebo Log"
  gz log -d 1
  echo -e "\e[32mINFO:\e[0m Launching Task1"
  

  roslaunch val_task1 task1.launch --screen > $data_log2 
  # stdbuf -oL roslaunch val_task2 task2.launch --screen --screen &> $data_log2

#  sleep 2100

  echo -e "\e[32mINFO:\e[0m Stop Gazebo Log"
  gz log -d 0
  echo -e "\e[32mINFO:\e[0m Killing Task2"
  killall roslaunch gzserver rosrun rostopic gzclient rosmaster rosout
  sleep 40

  echo -e "\e[32mINFO:\e[0m Copying gazbo log files"
  dir=`find ~/.gazebo/ -name state.log`
  state_log=$HOME'/indigo_ws/src/space_robotics_challenge/val_tasks/val_task1/log/state_'$current_time'.log'
  # filt_log='panel_detection_'$current_time'.log'
  cp $dir $state_log
#  echo -e "\e[32mINFO:\e[0m Filtering log files"
#  gz log -e -f $state_log --filter *.pose/*.pose -z 60 > $filt_log

  roscd srcsim/worlds
  erb t=1 unique.world.erb > unique_task1.world
done
