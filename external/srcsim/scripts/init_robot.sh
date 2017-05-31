#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "\e[32mINIT:\e[0m Wait 15s (ROS time) for system to load"
python $DIR/rossleep.py 15

echo -e "\e[32mINIT:\e[0m Lower harness"
rostopic pub -1 /valkyrie/harness/velocity std_msgs/Float32 '{data: -0.1}'

echo -e "\e[32mINIT:\e[0m Wait 5s (ROS time) to be lowered"
python $DIR/rossleep.py 3

echo -e "\e[32mINIT:\e[0m Switch to high level control"
rostopic pub -1 /ihmc_ros/valkyrie/control/low_level_control_mode ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage '{requested_control_mode: 2, unique_id: -1}'

echo -e "\e[32mINIT:\e[0m Wait 2s (ROS time) for control to be switched"
python $DIR/rossleep.py 2

echo -e "\e[32mINIT:\e[0m Detach from harness"
rostopic pub -1 /valkyrie/harness/detach std_msgs/Bool true &

if [ $1 = "true" ]; then
  echo -e "\e[32mINIT:\e[0m Start walking"
  rosrun srcsim walk_test.py
fi

echo -e "\e[32mINIT:\e[0m Done"
