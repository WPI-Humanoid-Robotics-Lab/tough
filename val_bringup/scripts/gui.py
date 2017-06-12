#!/usr/bin/env python

import sys
import socket
import subprocess
import select
import rospy
from PyQt4 import QtCore, QtGui, uic

qtCreatorFile = "rossrc.ui"
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)


# FIELD_IP = "192.168.2.10"
FIELD_IP = "localhost"
PORT = 8080

host = FIELD_IP
port = PORT

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(2)

# connect to remote host
try:
    s.connect((host, port))
except:
    print 'Unable to connect'
    sys.exit()

sys.stdout.write('[OCU] ');
sys.stdout.flush()

class SrcUI(QtGui.QMainWindow, Ui_MainWindow):

    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        Ui_MainWindow.__init__(self)
        self.setupUi(self)
        self.task1.clicked.connect(self.Task1)
        self.task2.clicked.connect(self.Task2)
        self.task2.clicked.connect(self.Task2)
        self.walkRotateLineEdit.returnPressed.connect(self.WalkRotate)
        self.walkStepsLineEdit2.returnPressed.connect(self.WalkSteps)
        
    def SendCommand(self, command):
        socket_list = [sys.stdin, s]
        # Get the list sockets which are readable
        ready_to_read, ready_to_write, in_error = select.select(socket_list, [], [])
        # print in_error
        # for sock in ready_to_read:
        #     print sock 
        #     print s
        #     if sock == s:
        #         # incoming message from remote server, s
        #         data = sock.recv(4096)
        #         if not data:
        #             print '\nDisconnected from braodcast server'
        #             sys.exit()
        #         else:
        #             pass
        #             # print data

        #     else:
        #         # user entered a message
        s.send(command)


    def Task1(self):
        print "Task1\n"
    	self.SendCommand("+roslaunch val_bringup final1.launch")

    def Task2(self):
    	print("Do Task 2")

    def Task2(self):
    	print("Do Task 2")

    def WalkRotate(self):
    	text = self.walkRotateLineEdit.text()
    	print "**Command Recieved** "
        command = "+runrun val_footstep walk_rotate "+str(text)
        # command = "+rostopic list"
        self.SendCommand(command)
    	self.walkRotateLineEdit.clear()

    def WalkSteps(self):
    	text = self.walkStepsLineEdit2.text()
    	print(text)
    	self.walkStepsLineEdit2.clear()

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    window = SrcUI()
    # text = dlg.ui.lineEdit.text()
    window.show()
    sys.exit(app.exec_())
