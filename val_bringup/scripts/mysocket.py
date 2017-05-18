#!/usr/bin/env python
import socket
import time
import rospy
from std_msgs.msg import String

FIELD_IP = "127.0.0.1" #""192.168.2.10"
PORT = 9009

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

def callback(data):
    #send only data that is required. it should be a string only
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    sock.mysend(data.data +" \n")

def listener():
    # Listen to the decision making topic
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)



if __name__ == '__main__':
    listener()

    while True:

        msg = sock.myreceive()
        if (msg[len(FIELD_IP)+4] == "+"):
            # any message that begins with a + is
            print msg[:len(FIELD_IP)+4] + "**Command Recieved** " + msg[len(FIELD_IP)+4:-1]
        else:
            print msg[:len(FIELD_IP)+4] + "**Message Recieved** " + msg[len(FIELD_IP)+4:-1]
