FROM ros:indigo-ros-base
# osrf/ros:indigo-desktop-full
LABEL maintainer "bpinaya@wpi.edu"

# USER whrl
# Setup home environment
# RUN useradd whrl
# RUN mkdir /home/whrl && chown -R whrl: /home/whrl
# RUN mkdir -p /home/whrl/go /home/whrl/bin /home/whrl/lib /home/whrl/include
# ENV PATH /home/whrl/bin:$PATH
# ENV PKG_CONFIG_PATH /home/whrl/lib/pkgconfig
# ENV LD_LIBRARY_PATH /home/whrl/lib

RUN export uid=1000 gid=1000 && \
    mkdir -p /home/whrl && \
    echo "whrl:x:${uid}:${gid}:Whrl,,,:/home/whrl:/bin/bash" >> /etc/passwd && \
    echo "whrl:x:${uid}:" >> /etc/group && \
    echo "whrl ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/whrl && \
    chmod 0440 /etc/sudoers.d/whrl && \
    chown ${uid}:${gid} -R /home/whrl

USER whrl
ENV HOME /home/whrl

# Installing general required packages
RUN sudo apt-get -y update && sudo apt-get install -y git \
	g++ \
	vim \
	wget \
	ca-certificates \
	ssh \
	&& sudo rm -rf /var/lib/apt/lists/

# add environment setup 
RUN /bin/bash -c "echo 'source /opt/ros/indigo/setup.bash' >> ~/.bashrc"
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget -O - http://packages.osrfoundation.org/gazebo.key | sudo apt-key add -
RUN sudo sh -c 'echo "deb http://srcsim.gazebosim.org/src trusty main" > /etc/apt/sources.list.d/src-latest.list'
RUN wget -O - http://srcsim.gazebosim.org/src/src.key | sudo apt-key add -
RUN wget -O - https://bintray.com/user/downloadSubjectPublicKey?username=bintray | sudo apt-key add -
RUN sudo apt-get -y update && sudo apt-get install -y srcsim



############### Not required as the srcsim and controller will be ona different server ###############
##Environment Variables
#RUN echo "Updating env variables for Java"
#RUN echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' >> ~/.bashrc
#RUN echo 'export IS_GAZEBO=true' >> ~/.bashrc
#
#RUN echo "Change owner ship of ihmc_ros_java_adapter"
#RUN sudo chmod -R 777 /opt/ros/indigo/share/ihmc_ros_java_adapter
#
#RUN echo "Copy ihmc ini file"
#RUN mkdir -p ${HOME}/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ${HOME}/.ihmc/IHMCNetworkParameters.ini
#
#RUN echo "Set real time prio for ihmc controller"
#RUN sudo bash -c 'echo "@ros - rtprio 99" > /etc/security/limits.d/ros-rtprio.conf'
#RUN sudo groupadd ros
#RUN sudo usermod -a -G ros root
############### Not required as the srcsim and controller will be ona different server ###############


RUN /bin/bash -c "source /opt/ros/indigo/setup.bash && \
                  mkdir -p ~/indigo_ws/src && \
                  cd ~/indigo_ws/src && \
                  catkin_init_workspace && \
                  cd ~/indigo_ws/ && \
                  catkin_make && \
                  echo 'source ~/indigo_ws/devel/setup.bash' >> ~/.bashrc"

RUN /bin/bash -c "source ~/.bashrc"

############### Not required as the srcsim and controller will be ona different server ###############
#RUN wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
#RUN mkdir -p ~/.gazebo/models
#
#RUN tar -xvf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
#RUN sudo rm /tmp/default.tar.gz
#
#RUN cd ~ && git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git
#RUN sudo mkdir /usr/lib/jdk
#RUN sudo sh -c 'cd /usr/lib/jdk && wget --no-check-certificate --no-cookies --header "Cookie: oraclelicense=accept-securebackup-cookie" http://download.oracle.com/otn-pub/java/jdk/8u112-b15/jdk-8u112-linux-x64.tar.gz && tar -xvzf jdk-8u112-linux-x64.tar.gz && sudo rm -rf jdk-8u112-linux-x64.tar.gz'
#RUN echo "export JAVA_HOME=/usr/lib/jdk/jdk1.8.0_112 export IHMC_SOURCE_LOCATION=~/ihmc-open-robotics-software" >> ~/.bashrc
#
#RUN echo source ~/.bashrc
#RUN export JAVA_HOME=/usr/lib/jdk/jdk1.8.0_112
#RUN export IHMC_SOURCE_LOCATION=~/ihmc-open-robotics-software
#
# Vinayak advice was not to use .gradlew, it was causing some issues while building it.
# RUN sh -c 'cd ~/ihmc-open-robotics-software ; git stash; git pull; git checkout origin/0.9-support; ./gradlew -q deployLocal'
# RUN sh -c 'cd ~/ihmc-open-robotics-software ; git stash'
# RUN sh -c 'cd ~/ihmc-open-robotics-software ; git pull'
# RUN sh -c 'cd ~/ihmc-open-robotics-software ; git checkout origin/0.9-support'
# RUN sh -c 'cd ~/ihmc-open-robotics-software ; ./gradlew -q deployLocal'
#
#RUN cd ~ && wget -P /tmp/ http://gazebosim.org/distributions/srcsim/valkyrie_controller.tar.gz
#RUN tar -xvf /tmp/valkyrie_controller.tar.gz -C ~
#RUN sudo rm /tmp/valkyrie_controller.tar.gz
#
# RUN cd ~/ihmc-open-robotics-software/
# RUN git stash
# RUN git pull
# RUN git checkout origin/0.9-support
# RUN ./gradlew -q deployLocal
#
#
#RUN mkdir ~/indigo_ws/src/ihmc_repos
#
# Clone the ihmc repos
#RUN cd ~/indigo_ws/src/ihmc_repos/ && git clone https://github.com/ihmcrobotics/ihmc-ros-control.git
#RUN cd ~/indigo_ws/src/ihmc_repos/ && git clone https://github.com/ihmcrobotics/ihmc_ros_core.git
#RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_ros_core && git checkout 0.9.0
############### Not required as the srcsim and controller will be ona different server ###############

# ros-indigo-joint-controller that one seems broken
RUN sudo apt-get install -y ruby ros-indigo-pcl-ros \
	ros-indigo-pcl-conversions ros-indigo-moveit \
	ros-indigo-trac-ik ros-indigo-footstep-planner \
	ros-indigo-humanoid-localization ros-indigo-multisense-ros \
	ros-indigo-laser-assembler ros-indigo-robot-self-filter \
	ros-indigo-tf2-geometry-msgs ros-indigo-joint-state-publisher \
	ros-indigo-octomap-server ros-indigo-octomap \
	ros-indigo-joint-trajectory-controller \
	ros-indigo-joint-state-controller ros-indigo-position-controllers \
	&& sudo rm -rf /var/lib/apt/lists/

RUN git clone https://github.com/ninja777/humanoid_navigation.git ~/indigo_ws/src/humanoid_navigation
RUN cd ~/indigo_ws/src/humanoid_navigation && git checkout indigo-devel
RUN sudo /bin/bash -c "source /opt/ros/indigo/setup.bash && cd ~/indigo_ws && catkin_make"

# Make ssh dir for gitlab deploy key and set up repo

RUN sudo mkdir /home/whrl/.ssh/
ADD ssh/id_rsa /home/whrl/.ssh/id_rsa
# RUN sudo chown whrl:whrl /home/whrl/.ssh -R
RUN sudo chown whrl:whrl /home/whrl/.ssh -R
RUN sudo touch /home/whrl/.ssh/known_hosts
RUN sudo /bin/sh -c "ssh-keyscan gitlab.com >> /home/whrl/.ssh/known_hosts"
RUN git clone git@gitlab.com:whrl/space_robotics_challenge.git ~/indigo_ws/src/space_robotics_challenge
RUN cd ~/indigo_ws/src/space_robotics_challenge && git submodule update --init --recursive

# RUN /bin/bash -c "source /opt/ros/indigo/setup.bash && cd ~/indigo_ws && catkin_make"
############### Not required as the srcsim and controller will be ona different server ###############
#RUN cd ~/indigo_ws/src/ihmc_repos/ihmc_ros_core/ && git apply ~/indigo_ws/src/space_robotics_challenge/val_common/patches/ihmc_ros_core.patch
############### Not required as the srcsim and controller will be ona different server ###############

# RUN /bin/bash -c "source /opt/ros/indigo/setup.bash && cd ~/indigo_ws && catkin_make"

#WHY `sudo rf -rf`
RUN /bin/bash -c "source /opt/nasa/indigo/setup.bash && cd ~/indigo_ws && sudo rm -rf devel/ build/ &&  catkin_make"

# RUN /bin/bash -c "cd ~/indigo_ws/ && source devel/setup.bash && roslaunch val_bringup final1.launch init:='false'"
# source devel/setup.bash
# roslaunch val_bringup final1.launch init:='false'

# This cmd should be the roslaunch val_bringup
CMD echo "Done"