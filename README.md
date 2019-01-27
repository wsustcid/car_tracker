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
```
An [RVIZ](http://wiki.ros.org/rviz) window will open showing the car and sensor output.
A gazebo window will appear showing the simulation.
Either use the controller to drive the prius around the world, or click on the gazebo window and use the `WASD` keys to drive the car.

If using a Logitech F710 controller:

* Make sure the MODE status light is off
* Set the swtich to XInput mode
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