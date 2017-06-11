#!/usr/bin/env python

import sys
import socket
import subprocess
import rospy
from PyQt4 import QtCore, QtGui, uic
 
qtCreatorFile = "rossrc.ui"
Ui_MainWindow, QtBaseClass = uic.loadUiType(qtCreatorFile)


# FIELD_IP = "192.168.2.10"
FIELD_IP = "localhost"
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
        
    def Task1(self):
    	print("Do Task 1")

    def Task2(self):
    	print("Do Task 2")

    def Task2(self):
    	print("Do Task 2")

    def WalkRotate(self):
    	text = self.walkRotateLineEdit.text()
    	print "**Command Recieved** "
        print "executing 'runrun val_footstep walk_rotate "+str(text) + "'"
        command = "+runrun val_footstep walk_rotate "+str(text)
        subprocess.Popen(command.split())
        # print("rosrun val_footstep walk_rotate "+ text)
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
