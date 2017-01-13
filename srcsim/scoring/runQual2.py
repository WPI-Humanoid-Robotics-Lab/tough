#!/usr/bin/python
import subprocess as sp
import os, sys
import shutil
import time
from time import sleep 
import signal

def handler(signum, frame):
    print 'Exiting!!!'
    sys.exit()

signal.signal(signal.SIGINT, handler)

if __name__ == "__main__":  
  # Delete older gazebo logs
  print "Killing gazebo and ros"
  killcommand = "killall roslaunch && killall gzserver"
  sp.call([os.getenv('SHELL'),'-i','-c', killcommand])
  time.sleep(2)
  
  try:
    shutil.rmtree('/home/whrl/.gazebo/log')
  except OSError:
    pass
  # Run the qual task launch file for 2.5 minutes 
  command1 = 'roslaunch val_bringup qual2.launch extra_gazebo_args:="-r" walk_test:=false'
  out = sp.Popen([os.getenv('SHELL'),'-i','-c', command1])
  # After 1.5 minutes, run rosrun val_manipulation val_qual2_node 
  time.sleep(190)
  out.kill()
  
  print "Killing gazebo and ros"
  killcommand = "killall roslaunch && killall gzserver"
  sp.call([os.getenv('SHELL'),'-i','-c', killcommand])
  
  # copy gazebo log to some directory 
  name = os.listdir('/home/'+os.getlogin()+'/.gazebo/log')[0]
  print "Logs are saved in "+name
  T = str(int(time.time()))
  
  shutil.copy2('/home/whrl/.gazebo/log/'+name+'/gzserver/state.log','/home/whrl/indigo_ws/src/space_robotics_challenge/srcsim/scoring/state'+T+'.log')
  
  print "Filtering the log file with postfix :"+T
  command3 = "gz log -e -f state"+T+".log --filter *.pose/*.pose -z 60 > Final"+T+".log"
  sp.call([os.getenv('SHELL'),'-i','-c', command3], cwd = '/home/whrl/indigo_ws/src/space_robotics_challenge/srcsim/scoring')
  
  
  command4 = "scoring_q2.rb Final"+T+".log"
  out = sp.call([os.getenv('SHELL'),'-i','-c', command4], cwd = '/home/whrl/indigo_ws/src/space_robotics_challenge/srcsim/scoring')
  print out
  
  
  
  


# filter the copied log file "gz log -e -f****" 
#run scoring script for task 2: `script_name filteredlog` 
#rename both log file and filtered log file by prefixing the run time 


