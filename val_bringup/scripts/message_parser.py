#!/usr/bin/env python

import socket
import time
import subprocess
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

import sched

FIELD_IP = "192.168.2.10"
# FIELD_IP = "localhost"
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
    sock.mysend(str(time.time()) + " : " + "Clicked point : x:"+str(data.x) +" y:"+ str(data.y) + " z:"+str(data.z) +" \n")

# counter = 0 
# countercheck = 0 
# resetflag = False

# def ihmc_callback(data):
#     print "we get here"
#     counter = counter + 1
#     if counter > 5000000 :
#         counter = 0
#         resetflag = True

# sch = sched.scheduler(time.time, time.sleep)

# def check_status(sc): 
#     print "Doing stuff..."
#     global counter
#     global countercheck
#     global resetflag
#     print countercheck
#     print counter
#     print resetflag

#     if counter > countercheck:
#         print " "
#     else:
#         sock.mysend("Probable the IHMC Controller Failed")

#     if resetflag == True:
#         resetflag = False
#         countercheck = 0

#     else:
#         countercheck = counter
    
#     sch.enter(5, 1, check_status, (sc,))

# sch.enter(5, 1, check_status, (sch,))
# sch.run()

def listener():
    # Listen to the decision making topic
    rospy.init_node('state_listener', anonymous=True)
    rospy.Subscriber("/field/log", String, state_callback)
    rospy.Subscriber("/clicked_point", Point, point_callback)
    # rospy.Subscriber("/ihmc_ros/valkyrie/output/imu/pelvis_pelvisMiddleImu", String, ihmc_callback)
    # rospy.Subscriber("/tf", String, ihmc_callback)

if __name__ == '__main__':
    listener()
    pub1 = rospy.Publisher('/decision_making/task1/events', String, queue_size=10)
    pub2 = rospy.Publisher('/decision_making/task2/events', String, queue_size=10)
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

        # rest of the messages
        else:
            print msg[:start_index] + "**Message Recieved** " + msg[start_index:-1]

