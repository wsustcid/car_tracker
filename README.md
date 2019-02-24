# Demo of Prius in ROS/GAZEBO

This is a simulation of a Prius in [gazebo 8](http://gazebosim.org) with sensor data being published using [ROS kinetic](http://wiki.ros.org/kinetic/Installation)
The car's throttle, brake, steering, and gear shifting are controlled by publishing a ROS message.
A ROS node allows driving with a gamepad or joystick.

# Video + Pictures

A video and screenshots of the demo can be seen in this blog post: https://www.osrfoundation.org/simulated-car-demo/

![Prius Image](https://www.osrfoundation.org/wordpress2/wp-content/uploads/2017/06/prius_roundabout_exit.png)

# Requirements

This demo has been tested on Ubuntu Xenial (16.04)

* An X server
* [Docker](https://www.docker.com/get-docker)
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)

# Recommended

* A joystick
* A joystick driver which creates links to `/dev/input/js0` or `/dev/input/js1`

This has been tested with the Logitech F710 in Xbox mode. If you have a different joystick you may need to adjust the parameters for the very basic joystick_translator node: https://github.com/osrf/car_demo/blob/master/car_demo/nodes/joystick_translator

# Building

First clone the repo, then run the script `build_demo.bash`.
It builds a docker image with the local source code inside.

```
$ cd car_demo
$ ./build_demo.bash
```

# Running

Connect a game controller to your PC.
Use the script `run_demo.bash` to run the demo.

```
$ ./run_demo.bash
source /workspace/devel/setup.bash

```
An [RVIZ](http://wiki.ros.org/rviz) window will open showing the car and sensor output.
A gazebo window will appear showing the simulation.
Either use the controller to drive the prius around the world, or click on the gazebo window and use the `WASD` keys to drive the car.

If using a Logitech F710 controller:

* Make sure the MODE status light is off
* Set the swtich to X Input mode
* The right stick controls throttle and brake
* The left stick controls steering
* Y puts the car into DRIVE
* A puts the car into REVERSE
* B puts the car into NEUTRAL



#### 测试记录

1. demo.launch 给完一个油门会保持速度，松开刹车后也会继续前进
2. 自动控制节点和遥控控制节点同时启动会收到两个速度，导致车速较慢
3. 使用monitor节点和自动控制节点车速较快，（貌似是一直加速？）油门给的是加速度？
4. 自动控制也是刹车始终给才会停下，不然车会一直前进, 如何让车停下？

## 1. File Description

```
tree -L 2
├── Dockerfile
├── build_demo.bash
└── run_demo.bash
├── prius_msgs
│   ├── CMakeLists.txt
│   ├── msg
│   └── package.xml
├── prius_description
│   ├── CMakeLists.txt
│   ├── meshes
│   ├── package.xml
│   └── urdf
├── car_demo
│   ├── CMakeLists.txt
│   ├── launch
│   ├── models
│   ├── nodes
│   ├── package.xml
│   ├── plugins
│   ├── rviz
│   └── worlds
├── README.md
```

### 1.1 Dockerfile

Dockerfile is a text file which describes the process of building our self defined image. In the dockerfile,  every instruction  is used to build a specific layer of the image.  From this dockerfile, we know that  the osrf/car_demo image will be build based on the osrf/ros:kinetic-desktop image.

### 1.2 build_demo.bash

The script which contains docker build instruction to build the osrf/car_demo image. Noting that this script and the Dockerfile should be placed under a same directory. 



### 1.3 run_demo.bash

Start a docker container using nvidia-docker

```
  # this container will be removed after it stops running.
  --rm=true \
  --name=carsim \
  # setup environment variables for multiple machine communication
  --env ROS_HOSTNAME=172.17.0.2 \
  --env ROS_MASTER_URI=http://192.168.31.253:11311 \
```

On the host computer

```
roscore
export ROS_HOSTNAME=ubuntu16
export ROS_MASTER_URI=http://192.168.31.253:11311
```



**Frequently used docker instructions:**

```
docker search []
docker pull [imagename:tag]
docker images
docker ps
docker ps -a

docker run --name [name] -it [image:tag] /bin/bash
docker inspect ws-docker
docker container stop
docker container start
docker exec -it ws-docker /bin/bash

docker rm [containername]
docker rmi [image]
```

### 1.4 prius_msgs

```
prius_msgs/
├── CMakeLists.txt
├── msg
│   └── Control.msg
└── package.xml
```

**The custom message type:**

```
Header header

# Range 0 to 1, 1 is max throttle
float64 throttle
# Range 0 to 1, 1 is max brake
float64 brake
# Range -1 to +1, +1 is maximum left turn
float64 steer

uint8 NO_COMMAND=0
uint8 NEUTRAL=1
uint8 FORWARD=2
uint8 REVERSE=3

uint8 shift_gears
```

### 1.5 prius_description

```
prius_description/
├── CMakeLists.txt
├── meshes
│   ├── hybrid_body.mtl
│   ├── hybrid_body.obj
│   ├── Hybrid_Interior.png
│   ├── Hybrid.png
│   ├── steering_wheel.mtl
│   ├── steering_wheel.obj
│   ├── wheel.mtl
│   ├── wheel.obj
│   └── Wheels3.png
├── package.xml
└── urdf
    └── prius.urdf
```

**prius.urdf**

```
## line 596
  <gazebo reference="front_camera_link">
    <sensor type="camera" name="front_camera_sensor">
      <update_rate>30.0</update_rate>
      <camera name="front_camera">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="front_camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>false</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>/prius/front_camera</cameraName>
        <imageTopicName>image_raw</imageTopicName>
        <cameraInfoTopicName>/prius/front_camera_info</cameraInfoTopicName>
        <frameName>camera_link</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

```

Image Topic: /prius/front_camera/image_raw

###  1.6 Car_demo

```
tree car_demo/ -L 2
car_demo/
├── CMakeLists.txt
├── launch
│   ├── demo.launch
│   └── monitor.launch
├── models
│   ├── cloverleaf_interchange
│   ├── construction_cone
│   ├── dumpster
│   ├── gas_station
│   ├── grey_wall
│   ├── house_1
│   ├── house_2
│   ├── house_3
│   ├── jersey_barrier
│   ├── mcity
│   ├── powerplant
│   └── speed_limit_sign
├── nodes
│   └── joystick_translator
├── package.xml
├── plugins
│   ├── gazebo_ros_block_laser.cpp
│   ├── gazebo_ros_block_laser.h
│   ├── PriusHybridPlugin.cc
│   └── PriusHybridPlugin.hh
├── rviz
│   └── demo.rviz
└── worlds
    └── mcity.world
```

**joystick_translatot.py**

```python
    def callback(self, message):
        rospy.logdebug("joy_translater received axes %s",message.axes)
        command = Control()
        command.header = message.header
        if message.axes[THROTTLE_AXIS] >= 0:
            command.throttle = message.axes[THROTTLE_AXIS]
            command.brake = 0.0
        else:
            command.brake = message.axes[THROTTLE_AXIS] * -1
            command.throttle = 0.0

        if message.buttons[3]:
            command.shift_gears = Control.FORWARD
        elif message.buttons[1]:
            command.shift_gears = Control.NEUTRAL
        elif message.buttons[0]:
            command.shift_gears = Control.REVERSE
        else:
            command.shift_gears = Control.NO_COMMAND

        command.steer = message.axes[STEERING_AXIS]
        self.last_published = message
        self.pub.publish(command)
```

**demo.launch**

```python
<?xml version="1.0"?>
<launch>
  <arg name="model" default="$(find prius_description)/urdf/prius.urdf"/>
  <arg name="rvizconfig" default="$(find car_demo)/rviz/demo.rviz" />

  <param name="robot_description" textfile="$(arg model)"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="verbose" value="true"/>
    <arg name="world_name" value="$(find car_demo)/worlds/mcity.world"/>
  </include>

  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" >
    <!-- <remap from="robot_description" to="different_robot_description" /> -->
    <!-- <remap from="joint_states" to="/prius/joint_states" /> -->
  </node>
  <node pkg="fake_localization" type="fake_localization" name="fake_localization">
    <!-- <remap from="base_pose_ground_truth" to="/prius/base_pose_ground_truth"/> -->
  </node>
  <node pkg="tf2_ros" type="static_transform_publisher" name="very_inaccurate_odom" args="0 0 0 0 0 0 odom base_link"/>
  <node pkg="car_demo" type="joystick_translator" name="joystick_translator"/>

  <!-- Run two joy nodes publishing to the same topic, just to cover two possible joystick locations -->
  <node pkg="joy" type="joy_node" name="joy_node0">
    <param name="dev" value="/dev/input/js0"/>
  </node>
  <node pkg="joy" type="joy_node" name="joy_node1">
    <param name="dev" value="/dev/input/js1"/>
  </node>

  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -x 3 -y -12 -z 0.5 -model prius"/>

  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />
</launch>
```



| /amcl_pose<br/>/base_pose_ground_truth
/clicked_point
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/initialpose
/joint_states
/move_base_simple/goal
/particlecloud
/prius
/prius/back_camera/image_raw
/prius/back_camera/parameter_descriptions
/prius/back_camera/parameter_updates
/prius/back_camera_info
/prius/back_sonar/left_far_range
/prius/back_sonar/left_middle_range
/prius/back_sonar/right_far_range
/prius/back_sonar/right_middle_range
/prius/center_laser/scan
/prius/front_camera/image_raw
/prius/front_camera/parameter_descriptions
/prius/front_camera/parameter_updates
/prius/front_camera_info
/prius/front_left_laser/scan
/prius/front_right_laser/scan
/prius/front_sonar/left_far_range
/prius/front_sonar/left_middle_range
/prius/front_sonar/right_far_range
/prius/front_sonar/right_middle_range
/prius/left_camera/image_raw
/prius/left_camera/parameter_descriptions
/prius/left_camera/parameter_updates
/prius/left_camera_info
/prius/right_camera/image_raw
/prius/right_camera/parameter_descriptions
/prius/right_camera/parameter_updates
/prius/right_camera_info
/rosout
/rosout_agg
/tf
/tf_static | ubuntu16@ubuntu16:~/catkin_ws$ rostopic list<br/>/clicked_point
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/gazebo_gui/parameter_descriptions
/gazebo_gui/parameter_updates
/initialpose
/move_base_simple/goal
/prius
/prius/back_camera/image_raw
/prius/back_camera_info
/prius/back_sonar/left_far_range
/prius/back_sonar/left_middle_range
/prius/back_sonar/right_far_range
/prius/back_sonar/right_middle_range
/prius/center_laser/scan
/prius/front_camera/image_raw
/prius/front_camera_info
/prius/front_left_laser/scan
/prius/front_right_laser/scan
/prius/front_sonar/left_far_range
/prius/front_sonar/left_middle_range
/prius/front_sonar/right_far_range
/prius/front_sonar/right_middle_range
/prius/left_camera/image_raw
/prius/left_camera_info
/prius/right_camera/image_raw
/prius/right_camera_info
/prius1/amcl_pose
/prius1/base_pose_ground_truth
/prius1/initialpose
/prius1/joint_states
/prius1/particlecloud
/prius1/prius/back_camera/image_raw
/prius1/prius/back_camera/parameter_descriptions
/prius1/prius/back_camera/parameter_updates
/prius1/prius/front_camera/image_raw
/prius1/prius/front_camera/parameter_descriptions
/prius1/prius/front_camera/parameter_updates
/prius1/prius/left_camera/image_raw
/prius1/prius/left_camera/parameter_descriptions
/prius1/prius/left_camera/parameter_updates
/prius1/prius/right_camera/image_raw
/prius1/prius/right_camera/parameter_descriptions
/prius1/prius/right_camera/parameter_updates
/prius2/amcl_pose
/prius2/base_pose_ground_truth
/prius2/initialpose
/prius2/joint_states
/prius2/particlecloud
/prius2/prius/back_camera/image_raw
/prius2/prius/back_camera/parameter_descriptions
/prius2/prius/back_camera/parameter_updates
/prius2/prius/front_camera/image_raw
/prius2/prius/front_camera/parameter_descriptions
/prius2/prius/front_camera/parameter_updates
/prius2/prius/left_camera/image_raw
/prius2/prius/left_camera/parameter_descriptions
/prius2/prius/left_camera/parameter_updates
/prius2/prius/right_camera/image_raw
/prius2/prius/right_camera/parameter_descriptions
/prius2/prius/right_camera/parameter_updates
/rosout
/rosout_agg
/tf
/tf_static |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
|                                                              |                                                              |

