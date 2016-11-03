#!/bin/bash

echo "wait 60s for system to load"
sleep 60

echo "lower harness"
rostopic pub -1 /valkyrie/harness/velocity std_msgs/Float32 '{data: -0.05}'

echo "switch to high level control after 20 seconds"
sleep 20
rostopic pub -1 /ihmc_ros/valkyrie/control/low_level_control_mode ihmc_valkyrie_ros/ValkyrieLowLevelControlModeRosMessage '{requested_control_mode: 2, unique_id: -1}'

echo "detach in 5 seconds"
sleep 5
rostopic pub -1 /valkyrie/harness/detach std_msgs/Bool true &

if [ $1 = "true" ]; then
  echo "start walking"
  rosrun srcsim walk_test.py
fi

echo "done"
