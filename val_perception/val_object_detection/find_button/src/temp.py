#!/usr/bin/env python
import rospy
from stereo_msgs.msg import DisparityImage

def callback(info):
	print info.f

if __name__ == '__main__':
	rospy.init_node('temp', anonymous=True)
	rospy.Subscriber("/multisense/camera/disparity", DisparityImage, callback)
	rospy.spin()
