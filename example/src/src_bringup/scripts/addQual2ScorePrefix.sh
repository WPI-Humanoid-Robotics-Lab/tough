#!/usr/bin/bash

source ~/indigo_ws/devel/setup.bash
logFiles=`find ./test/ -name "qual*"`

for filt_log in $logFiles
do
  filt_log=${filt_log#*/}
  echo -e "\e[32mINFO:\e[0m Scoring the log"
  roscd srcsim/scoring
  score=`./scoring_q2.rb $filt_log`
  
    
  echo -e "\e[32mINFO:\e[0m Renaming based on score"
  # TODO: get a better extraction of the score ..!!!!!!!!
  echo "Scoring results :"$score
  score="${score#*:}"     # Remove through first :
  score="${score#*:}"   # Remove through second :
  score="${score#*:}"   # Remove through second :
  score="${score#*[}"
  score=${score/ /.}
  score=${score:0:5}
   
  echo "Score :"$score
  echo $score
  if  [ "$score" == ".--" ]; then
    mv $filt_log 'test/9999'${filt_log#*/}
  else
    echo $filt_log 'test/'$score${filt_log#*/}
    mv $filt_log 'test/'$score${filt_log#*/}
  fi
done