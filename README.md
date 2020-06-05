# T.O.U.G.H
Transportable Opensource API & UI for Generic Humanoids
<p align="left"> 
 <a href="https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/LICENSE" alt="Contributors">
        <img src="https://img.shields.io/apm/l/vim-mode.svg?style=popout" /></a>
 <a href="https://github.com/WPI-Humanoid-Robotics-Lab/tough/issues" alt="Contributors">
        <img src="https://img.shields.io/github/issues/WPI-Humanoid-Robotics-Lab/tough.svg" /></a>
</p>


![logo](./docs/logo.png)


TOUGH library provides C++ API to humanoid robots supported by IHMC controllers. The code is tested on Valkyrie R5 (branch: ihmc-0.8.2), srcsim (tag: 0.1a), Atlas/Valkyrie in SCS simulator(0.11.0) and the Atlas robot(0.11.0) . The branch kinetic-0.11.0 is in active development and may not work as expected. ihmc_msgs are different for different versions. Following table gives the corresponding tough tag/branch name for versions of ihmc repo


| ihmc controller version | tough             |
|:-----------------------:| ------------------|
| 0.8.2                   | ihmc-0.8.2        | 
| 0.9.x                   | 0.1a              | 
| 0.11                    | 0.11.0            |  

We are skipping 0.10 version of ihmc controllers more details [here](https://github.com/ihmcrobotics/ihmc-open-robotics-software/issues/133)

### Installation
 - Requires: Ubuntu 16.04, ROS-Kinetic
 - [tough-kinetic-installation-instructions](https://github.com/WPI-Humanoid-Robotics-Lab/tough/wiki/Tough-0.11-Installation-using-vcstool)

### Tutorials
- Control examples
  - [arm example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/arm_control_example.cpp)
  - [arm example 2](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/arm_control_example2.cpp)
  - [interactive arm example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/arm_interactive_example.cpp)
  - [chest example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/chest_control_example.cpp)
  - [gripper control example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/wiki/GripperController-example)
  - [head example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/head_control_example.cpp)
  - [neck example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/neck_control_example.cpp)
  - [nudge local example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/nudge_local_example.cpp)
  - [nudge offset example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/nudge_offset_example.cpp)
  - [pelvis example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/pelvis_control_example.cpp)
  - [reset robot](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/control_examples/reset_robot.cpp)
- Manipulation examples
  - [motion planner example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/manipulation_examples/motion_planner_example.cpp)
  - [taskspace planner example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/manipulation_examples/taskspace_planner_example.cpp)
- Navigation examples
  - [walk example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/navigation_examples/walk_example.cpp)
  - [walk forward example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/navigation_examples/walk_forward_example.cpp)
  - [walk to goal example](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/navigation_examples/walk_goal_example.cpp)
  - [walk rotate](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/navigation_examples/walk_rotate_example.cpp)
  - [walk step](https://github.com/WPI-Humanoid-Robotics-Lab/tough/blob/kinetic-0.11.0/tough_examples/src/navigation_examples/walk_steps_example.cpp)
- Perception examples
  - [MultisenseImage](https://github.com/WPI-Humanoid-Robotics-Lab/tough/wiki/MultisenseImage-example)

### Citation
If you find this work useful in your research, please consider to cite:
```
@article{Jagtap2020,
    author = {Jagtap, Vinayak and Agarwal, Shlok and Wagh, Ameya and Gennert, Michael},
    journal = {Int. J. Adv. Robot. Syst.},
    keywords = {Humanoid robots,ROS,open-source projects and applications for robotics},
    title = {{Transportable open-source application program interface and user interface for generic humanoids: TOUGH}},
    volume = {17},
    year = {2020}
}
```
### Checkout [Wiki](https://github.com/WPI-Humanoid-Robotics-Lab/tough/wiki) for further instructions
