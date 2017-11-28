* val_moveit_tasks : 
This package has raise_arm_move_group node that can be launched using raise_arm.launch. The raise_arm_move_group node raise left arm of the robot when button_detected signal is True. 


val_pose
uses the moveit! and val_control libraries to pose the robot's arms to an interactive marker.
launch group_move, and val_pose to operate  


TODO:

Following are instructions to run the codes successfully
In a new terminal,
```bash 
rostopic pub button_detected std_msgs/Bool true
```
In another terminal,
```bash 
roslaunch val_moveit_tasks raise_arm.launch
```