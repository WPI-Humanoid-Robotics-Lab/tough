#!/usr/bin/bash

terminate ()
{
echo 'killed'
kill -s SIGTERM $!
exit 0
}

trap terminate SIGINT SIGTERM

while true
do
  echo -e "\e[32mINFO:\e[0m Killing exisitng ros/gazebo instances"
  killall roslaunch && killall gzserver && killall rosmaster && killall rosout

  echo -e "\e[32mINFO:\e[0m Removing old log files"
  rm -rf ~/.gazebo/log/*

  echo -e "\e[32mINFO:\e[0m Launching qual2"
  roslaunch val_bringup qual2.launch extra_gazebo_args:="-r" walk_test:=false &
     
  echo -e "\e[32mINFO:\e[0m Sleeping for 180 seconds"
  sleep 190
     
  echo -e "\e[32mINFO:\e[0m Killing qual2"
  killall roslaunch
  killall gzserver

  echo -e "\e[32mINFO:\e[0m Copying log files"
  source ~/indigo_ws/devel/setup.bash
  roscd srcsim/scoring/test/
  dir=`find /home/ninja/.gazebo/ -name state.log`
  current_time=$(date "+%Y.%m.%d-%H.%M.%S")
  state_log='state_'$current_time'.log'
  filt_log='qual2_'$current_time
  cp $dir $state_log

  echo -e "\e[32mINFO:\e[0m Filtering log files"
  gz log -e -f $state_log --filter *.pose/*.pose -z 60 > $filt_log'.log'
     
  echo -e "\e[32mINFO:\e[0m Scoring the log"
  cd ..
  score=`./scoring_q2.rb 'test\'$filt_log'.log'`
  cd test
     
  echo -e "\e[32mINFO:\e[0m Renaming based on score"
  # TODO: get a better extraction of the score ..!!!!!!!!
  echo $score
  score="${score#*:}"     # Remove through first :
  score="${score#*:}"   # Remove through second :
  score="${score#*:}"   # Remove through second :
  score="${score%%:*}"
  echo $score
  if  [ "$score" == " --" ]; then
    mv $filt_log'.log' '9999'$filt_log'.log'
  else
    mv $filt_log'.log' $score$filt_log'.log'
  fi

done
