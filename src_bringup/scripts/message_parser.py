#!/usr/bin/env python

import socket
import time
import subprocess
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from ihmc_msgs.msg import AbortWalkingRosMessage
from ihmc_msgs.msg import StopAllTrajectoryRosMessage
import os

FIELD_IP = os.environ.get('FIELD_IP'):

if !FIELD_IP:
  FIELD_IP = "localhost"
  print "Connecting to local server"

PORT = 8080

class mysocket:
    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(
                socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def connect(self, host, port):
        self.sock.connect((host, port))

    def mysend(self, msg):
        totalsent = 0
        while totalsent < len(msg):
            sent = self.sock.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def myreceive(self):
        chunks = []
        bytes_recd = 0
        proceed = True
        while proceed:
            chunk = self.sock.recv(4096)
            if chunk == '':
                raise RuntimeError("socket connection broken")
            if (chunk.endswith("\n")):
                proceed = False
            chunks.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
        return ''.join(chunks)

sock = mysocket()
sock.connect(FIELD_IP,PORT)

def state_callback(data):
    #send only data that is required. it should be a string only
    rospy.loginfo(rospy.get_caller_id() + "Message %s", data.data)
    sock.mysend(str(time.time()) + " : " + data.data +" \n")

def point_callback(data):
    #send only data that is required. it should be a string only
    sock.mysend(str(time.time()) + " : " + "Clicked point : x:"+str(data.point.x) +" y:"+ str(data.point.y) + " z:"+str(data.point.z) +" \n")

def listener():
    # Listen to the decision making topic
    rospy.init_node('state_listener', anonymous=True)
    rospy.Subscriber("/field/log", String, state_callback)
    rospy.Subscriber("/clicked_point", PointStamped, point_callback)
    # rospy.Subscriber("/ihmc_ros/valkyrie/output/joint_states", JointState, ihmc_callback)
    # rospy.Subscriber("/tf", String, ihmc_callback)

if __name__ == '__main__':
    listener()
    pub1 = rospy.Publisher('/decision_making/task1/events', String, queue_size=10)
    pub2 = rospy.Publisher('/decision_making/task2/events', String, queue_size=10)
    panel_offset_pub = rospy.Publisher('/panel_offset', Float32, queue_size=10)
    nudge_pub  = rospy.Publisher('/nudge_pose', Float64MultiArray, queue_size=10)
    head_pub   = rospy.Publisher('/head_control', Float32MultiArray, queue_size=10)
    abort_walk_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/abort_walking', AbortWalkingRosMessage, queue_size=1)
    stop_all_traj_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/stop_all_trajectories', StopAllTrajectoryRosMessage, queue_size=1)
    prev_time = time.time()
    prev_ros_time = rospy.get_rostime().secs
    while True:
        current_time = time.time()

        if current_time - prev_time > 5:
            ros_time = rospy.get_rostime().secs
            rtf = (ros_time - prev_ros_time)/ (current_time - prev_time)
            prev_time = time.time()
            prev_ros_time = rospy.get_rostime().secs
            sock.mysend(str(time.time()) + " : " + "Real Time Factor : "+ str(rtf)+"\n")
        msg = sock.myreceive()
        start_index = msg.find("]")

        if start_index == -1:
            continue
        start_index += 2

        # + is a linux command to be executed
        if (msg[start_index] == "+"):
            command = msg[start_index+1:-1]
            print msg[:start_index] + "**Command Recieved** " + msg[start_index:-1]
            print "executing '"+command + "'"
            subprocess.Popen(command.split())

        # ! is rostopic echo for 1 message.
        elif (msg[start_index] == "!"):
            topic_name = msg[start_index + 1:-1]
            print msg[:start_index] + "Listening to " + msg[start_index:-1]
            proc = subprocess.Popen(["rostopic", "echo", "-n", "1", topic_name], stdout=subprocess.PIPE)
            output = proc.stdout.read()
            sock.mysend(str(time.time()) + " : " +  output + "\n")

        # / is message to be sent to the state machine
        elif (msg[start_index] == "/"):
            print msg[:start_index] + "**Message Recieved** " + msg[start_index:-1]
            pub1.publish(msg[start_index:-1])
            pub2.publish(msg[start_index:-1])
        
        # $ is message to be sent to the state machine
        elif (msg[start_index] == "$"):
            print msg[:start_index] + "**offset received** " + msg[start_index:-1]
            panel_offset_pub.publish(float(msg[start_index + 1 :-1]))

        # * is message to be sent to the nudge node
        elif (msg[start_index] == "*"):
            print msg[:start_index] + "**Nudge message received** " + msg[start_index:-1]
            data_str = msg[start_index + 1:-1].split()
            ros_msg = Float64MultiArray()
            ros_msg.data = [float(x) for x in data_str]
            nudge_pub.publish(ros_msg)

        # * is message to be sent to the head node
        elif (msg[start_index] == "&"):
            print msg[:start_index] + "**Head control message received** " + msg[start_index:-1]
            data_str = msg[start_index + 1:-1].split()
            ros_msg = Float32MultiArray()
            ros_msg.data = [float(x) for x in data_str]
            print ros_msg.data
            head_pub.publish(ros_msg)

        # | is message to abort walking
        elif (msg[start_index] == "|"):
            print msg[:start_index] + "**Abort walking message received** " + msg[start_index:-1]
            ros_msg = AbortWalkingRosMessage()
            ros_msg.unique_id = 1
            abort_walk_pub.publish(ros_msg)

        # _ is message to stop all trajectories
        elif (msg[start_index] == "_"):
            print msg[:start_index] + "**Stopping all trajectories** " + msg[start_index:-1]
            ros_msg = StopAllTrajectoryRosMessage()
            ros_msg.unique_id = 1
            stop_all_traj_pub.publish(ros_msg)

        # rest of the messages
        else:
            print msg[:start_index] + "**Message Recieved** " + msg[start_index:-1]

