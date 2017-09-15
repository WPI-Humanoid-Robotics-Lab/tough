#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from srcsim.msg import Harness
import subprocess
import time

execute = True

def callback(data):
    global execute
    #robot detached from harness, wait for 5 sec and start all the nodes
    if data.status == 5 and execute:
      pub = rospy.Publisher('/multisense/set_spindle_speed', Float64, queue_size=10)
      time.sleep(1)
      pub.publish(0.8)
#      subprocess.Popen(["roslaunch", "val_perception_bringup", "field_laser_assembler.launch"])
#      subprocess.Popen(["roslaunch", "val_perception_bringup", "field_octomap.launch"])
      subprocess.Popen(["roslaunch", "val_footstep", "val_footstep.launch"])
      subprocess.Popen(["roslaunch", "val_moveit_config", "move_group.launch"])
      subprocess.Popen(["rosrun", "navigation_common", "fall_detector"])
      execute = False

def WaitForRobot():
    rospy.init_node('WaitForRobot', anonymous=True)

    rospy.Subscriber("/srcsim/finals/harness", Harness, callback)
    # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    global execute
    WaitForRobot()
    time.sleep(180)
    if (not execute):
        rospy.spin()
    else:
        subprocess.Popen(["killall", "roslaunch", "roscore", "rosout", "gzserver", "gzclient"])
