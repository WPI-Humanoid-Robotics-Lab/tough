#!/usr/bin/bash

terminate()
{
echo 'killed'
kill -s SIGTERM $!
exit 0
}

trap terminate SIGINT SIGTERM


while true
do
  echo -e "\e[32mINFO:\e[0m Sourcing" 
  source ~/indigo_ws/devel/setup.bash

  echo -e "\e[32mINFO:\e[0m Killing exisitng ros/gazebo instances"
  killall roslaunch && killall gzserver && killall rosmaster && killall rosout

  echo -e "\e[32mINFO:\e[0m Removing old log files"
  rm -rf ~/.gazebo/log/*
  rm ~/src_qual1* 
  
  echo -e "\e[32mINFO:\e[0m Changing Directory" 
  roscd val_bringup
  echo -e "\e[32mINFO:\e[0m Launching qual1"
  roslaunch val_bringup qual1.launch extra_gazebo_args:="-r" init:=true &
  sleep 90
  rosrun led_detector led_detector &
  sleep 10
  rostopic pub /srcsim/qual1/start std_msgs/Empty &
     
  echo -e "\e[32mINFO:\e[0m Sleeping for 180 seconds"
  sleep 30
     
  echo -e "\e[32mINFO:\e[0m Killing qual2"
  killall roslaunch
  killall gzserver

  echo -e "\e[32mINFO:\e[0m Copying log files"
  roscd srcsim/scoring
  
  
  dir=`find ~/.gazebo/ -name state.log`
  dataFile=`find ~/ -maxdepth 1 -name src_qual1*`
  current_time=$(date "+%Y.%m.%d-%H.%M.%S")
  data_log = 'data_'$current_time'.log'
  state_log='state_'$current_time'.log'
  filt_log='qual1_'$current_time'.log'
  cp $datafile $data_log
  cp $dir $state_log

  echo -e "\e[32mINFO:\e[0m Filtering log files"
  gz log -e -f $state_log --filter *.pose/*.pose -z 60 > $filt_log
     
  echo -e "\e[32mINFO:\e[0m Scoring the log"
  cd ..
  score=`./scoring_q1.rb $data_log $filt_log`
  cd test
     
  echo -e "\e[32mINFO:\e[0m Renaming based on score"
  # TODO: get a better extraction of the score ..!!!!!!!!
  echo $score
  
  score="${score#*Total position euclidean error (head): }"     # Remove through first :
  score=`echo $score | sed 's/[[:space:]]//g'`
  score=${score:0:5}                                            # Take only first 4 characters
  
  echo $score
  if  [ "$score" == ".--" ]; then
    mv $filt_log '9999'$filt_log
    mv $state_log '9999'$state_log
    mv $data_log'9999'$data_log
  else
    mv $filt_log $score$filt_log
    mv $state_log $score$state_log
    mv $data_log $score$data_log
  fi

done
