#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "\e[32mINIT:\e[0m Wait 15s (ROS time) for system to load"
python $DIR/rossleep.py 15

echo -e "\e[32mINIT:\e[0m Lower harness"
rostopic pub -1 /valkyrie/harness/velocity std_msgs/Float32 '{data: -0.05}'

echo -e "\e[32mINIT:\e[0m Wait 5s (ROS time) to be lowered"
python $DIR/rossleep.py 5

echo -e "\e[32mINIT:\e[0m Switch to high level control"
rostopic pub -1 /ihmc_ros/valkyrie/control/low_level_control_mode ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage '{requested_control_mode: 2, unique_id: -1}'

echo -e "\e[32mINIT:\e[0m Wait 2s (ROS time) for control to be switched"
python $DIR/rossleep.py 2

echo -e "\e[32mINIT:\e[0m Detach from harness"
rostopic pub -1 /valkyrie/harness/detach std_msgs/Bool true &
python $DIR/rossleep.py 6

if [ $1 = "true" ]; then
  echo -e "\e[32mINIT:\e[0m Start walking"
  rosrun srcsim walk_test.py
fi

gzprocess=`pgrep -x "gzserver"`
counter=`echo "$gzprocess" | wc -l`

#if [ $counter -ne 2 ]
#then
#  echo -e "\e[32mINIT:\e[0m gzserver process not found. Restarting!"
#  killall roslaunch
#  exit -1
#fi

python $DIR/rossleep.py 6

# #comment bellow for rosbags
# echo -e "\e[32mINIT:\e[0m Starting the lights"
# rostopic pub -1 /srcsim/qual1/start std_msgs/Empty &

# echo -e "\e[32mINIT:\e[0m Start Qual1"
# rosrun led_detector led_detector 10 # input k value

# echo -e "\e[32mINIT:\e[0m Done"
# killall roslaunch
