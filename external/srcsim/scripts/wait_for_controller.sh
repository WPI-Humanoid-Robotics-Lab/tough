#!/bin/sh

while [ -z "$(rosparam list | grep ihmc_valkyrie_control_java_bridge)" ]; do
  echo "waiting for ihmc_valkyrie_control_java_bridge"
  sleep 1
done

echo "Found ihmc_valkyrie_control_java_bridge! Launching grasp_init."

sleep 5

roslaunch srcsim grasping_init.launch
