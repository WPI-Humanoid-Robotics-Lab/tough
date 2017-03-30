#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import subprocess
import time

def callback(data):
    #robot detached from harness, wait for 5 sec and start all the nodes
    if data.data:
      pub = rospy.Publisher('/multisense/set_spindle_speed', Float64, queue_size=10)
      time.sleep(1)
      subprocess.Popen(["roslaunch", "val_grasping", "val_grasping.launch"])
      pub.publish(0.55)
      subprocess.Popen(["roslaunch", "val_perception_bringup", "laser_assembler.launch"])
      subprocess.Popen(["roslaunch", "val_perception_bringup", "octomap.launch"])
      subprocess.Popen(["roslaunch", "val_footstep", "val_footstep.launch"])
    
def WaitForRobot():
    rospy.init_node('WaitForRobot', anonymous=True)

    rospy.Subscriber("/valkyrie/harness/detach", Bool, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    WaitForRobot()
