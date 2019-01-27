FROM osrf/ros:kinetic-desktop

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN apt-get update \
 && apt-get install -y \
    wget \
    lsb-release \
    sudo \
    mesa-utils \
 && apt-get clean


# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    gazebo8 \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-fake-localization \
    ros-kinetic-joy \
 && apt-get clean

# edited by ws
RUN mkdir -p /workspace/src
COPY prius_description /workspace/src/prius_description
COPY prius_msgs /workspace/src/prius_msgs
COPY car_demo /workspace/src/car_demo
RUN /bin/bash -c 'cd /workspace \
 && source /opt/ros/kinetic/setup.bash \
 && catkin_make'


#CMD ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && source /workspace/devel/setup.bash"]
#&& roslaunch car_demo demo.launch