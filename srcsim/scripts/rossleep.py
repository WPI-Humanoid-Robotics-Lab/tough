#!/usr/bin/env python
import rospy
import sys

if len(sys.argv) != 2:
    print("Proper usage: %s <float>" % sys.argv[0])
    sys.exit(1)

try:
    seconds = float(sys.argv[1])
except (ValueError):
    print("Couldn't convert argument [%s] to float" % sys.argv[1])
    sys.exit(1)

rospy.init_node('sleeper', anonymous=True)
rospy.sleep(rospy.Duration.from_sec(seconds))
