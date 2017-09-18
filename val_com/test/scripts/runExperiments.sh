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
  if [ ! -d ~/indigo_ws/src/space_robotics_challenge/val_com/test/log ]; then
    mkdir -p ~/indigo_ws/src/space_robotics_challenge/val_com/test/log ;
  fi

  echo -e "\e[32mINFO:\e[0m Sourcing"
  source ~/indigo_ws/devel/setup.bash
  echo -e "\e[32mINFO:\e[0m Killing exisitng ros/gazebo instances"
  killall roslaunch gzserver rosmaster rosout gzclient
  rosclean purge -y
  sleep 15

  # roscd val_common/scripts/task2
  echo -e "\e[32mINFO:\e[0m Launching SRCSIM"
  roslaunch val_bringup final1.launch &

  sleep 150
  rosrun test test_com_stability_node >/dev/null 2>/dev/null &

  rosrun test com_experiment
  echo -e "\e[32mINFO:\e[0m Killing exisitng ros/gazebo instances"
  killall roslaunch gzserver rosmaster rosout gzclient
  sleep 30

done
