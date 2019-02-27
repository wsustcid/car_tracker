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



PriusHybridPlugin.cc

```c++
namespace gazebo
{
  class PriusHybridPluginPrivate
  {
    /// \enum DirectionType
    /// \brief Direction selector switch type.
    public: enum DirectionType {
              /// \brief Reverse
              REVERSE = -1,
              /// \brief Neutral
              NEUTRAL = 0,
              /// \brief Forward
              FORWARD = 1
            };

    public: ros::NodeHandle nh;

    public: ros::Subscriber controlSub;
    public: ignition::transport::Node::Publisher posePub;
    /// \brief Ignition transport console pub
    public: ignition::transport::Node::Publisher consolePub;
      
    /// \brief Current direction of the vehicle: FORWARD, NEUTRAL, REVERSE.
    public: DirectionType directionState;

    /// \brief Chassis aerodynamic drag force coefficient,
    /// with units of [N / (m/s)^2]
    public: double chassisAeroForceGain = 0;

    /// \brief Max torque that can be applied to the front wheels
    public: double frontTorque = 0;

    /// \brief Max torque that can be applied to the back wheels
    public: double backTorque = 0;

    
    /// \brief Max torque that can be applied to the front brakes
    public: double frontBrakeTorque = 0;

    /// \brief Max torque that can be applied to the rear brakes
    public: double backBrakeTorque = 0;


    /// \brief Front left joint friction
    public: double flJointFriction = 0;

    /// \brief Front right joint friction
    public: double frJointFriction = 0;

    /// \brief Rear left joint friction
    public: double blJointFriction = 0;

    /// \brief Rear right joint friction
    public: double brJointFriction = 0;


    /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
    public: ignition::math::Vector3d chassisLinearVelocity;

  
    /// \brief Subscriber to the keyboard topic
    public: transport::SubscriberPtr keyboardSub;


    /// \brief Odometer
    public: double odom = 0.0;

    /// \brief Keyboard control type
    public: int keyControl = 1;

    /// \brief Publisher for the world_control topic.
    public: transport::PublisherPtr worldControlPub;
  };
}

using namespace gazebo;

/////////////////////////////////////////////////
PriusHybridPlugin::PriusHybridPlugin()
    : dataPtr(new PriusHybridPluginPrivate)
{
  ros::NodeHandle nh;
  this->dataPtr->controlSub = nh.subscribe("prius", 10, &PriusHybridPlugin::OnPriusCommand, this);
  // Default is forward
  this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
}

/// achieves user control commmand convert to plugin's command: 
/// handWheelCmd, brakePedalPercent, gasPedalPercent, directionState
void PriusHybridPlugin::OnPriusCommand(const prius_msgs::Control::ConstPtr &msg)
{
  // Steering wheel command
  double handCmd = (msg->steer < 0.)
    ? (msg->steer * -this->dataPtr->handWheelLow)  //?? default is 0
    : (msg->steer * this->dataPtr->handWheelHigh);

  handCmd = ignition::math::clamp(handCmd, this->dataPtr->handWheelLow,
      this->dataPtr->handWheelHigh);
  this->dataPtr->handWheelCmd = handCmd;

  // Brake command
  double brake = ignition::math::clamp(msg->brake, 0.0, 1.0);
  this->dataPtr->brakePedalPercent = brake;

  // Throttle command
  double throttle = ignition::math::clamp(msg->throttle, 0.0, 1.0);
  this->dataPtr->gasPedalPercent = throttle;

  switch (msg->shift_gears)
  {
    case prius_msgs::Control::NEUTRAL:
      this->dataPtr->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    case prius_msgs::Control::FORWARD:
      this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
      break;
    case prius_msgs::Control::REVERSE:
      this->dataPtr->directionState = PriusHybridPluginPrivate::REVERSE;
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
PriusHybridPlugin::~PriusHybridPlugin()
{
  this->dataPtr->updateConnection.reset();
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "PriusHybridPlugin loading params" << std::endl;
  // shortcut to this->dataPtr
  PriusHybridPluginPrivate *dPtr = this->dataPtr.get();

  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();
  auto physicsEngine = this->dataPtr->world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  this->dataPtr->gznode = transport::NodePtr(new transport::Node());
  this->dataPtr->gznode->Init();

  this->dataPtr->node.Subscribe("/prius/reset",
      &PriusHybridPlugin::OnReset, this);
  this->dataPtr->node.Subscribe("/prius/stop",
      &PriusHybridPlugin::OnStop, this);

  this->dataPtr->node.Subscribe("/cmd_vel", &PriusHybridPlugin::OnCmdVel, this);
  this->dataPtr->node.Subscribe("/cmd_gear",
      &PriusHybridPlugin::OnCmdGear, this);
  this->dataPtr->node.Subscribe("/cmd_mode",
      &PriusHybridPlugin::OnCmdMode, this);

  this->dataPtr->posePub = this->dataPtr->node.Advertise<ignition::msgs::Pose>(
      "/prius/pose");
  this->dataPtr->consolePub =
    this->dataPtr->node.Advertise<ignition::msgs::Double_V>("/prius/console");


  std::string paramName;
  double paramDefault;

  paramName = "chassis_aero_force_gain";
  paramDefault = 1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->chassisAeroForceGain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->chassisAeroForceGain = paramDefault;

  paramName = "front_torque";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontTorque = paramDefault;

  paramName = "back_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backTorque = paramDefault;

  paramName = "front_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontBrakeTorque = paramDefault;

  paramName = "back_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backBrakeTorque = paramDefault;

  paramName = "battery_charge_watt_hours";
  paramDefault = 280;
  if (_sdf->HasElement(paramName))
    this->dataPtr->batteryChargeWattHours = _sdf->Get<double>(paramName);
  else
    this->dataPtr->batteryChargeWattHours = paramDefault;

  paramName = "battery_discharge_watt_hours";
  paramDefault = 260;
  if (_sdf->HasElement(paramName))
    this->dataPtr->batteryDischargeWattHours = _sdf->Get<double>(paramName);
  else
    this->dataPtr->batteryDischargeWattHours = paramDefault;

  paramName = "gas_efficiency";
  paramDefault = 0.37;
  if (_sdf->HasElement(paramName))
    this->dataPtr->gasEfficiency = _sdf->Get<double>(paramName);
  else
    this->dataPtr->gasEfficiency = paramDefault;

  paramName = "min_gas_flow";
  paramDefault = 1e-4;
  if (_sdf->HasElement(paramName))
    this->dataPtr->minGasFlow = _sdf->Get<double>(paramName);
  else
    this->dataPtr->minGasFlow = paramDefault;

  paramName = "max_speed";
  paramDefault = 10;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSpeed = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSpeed = paramDefault;

  paramName = "max_steer";
  paramDefault = 0.6;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSteer = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSteer = paramDefault;

  paramName = "flwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetPGain(paramDefault);

  paramName = "frwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetPGain(paramDefault);

  paramName = "flwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetIGain(paramDefault);

  paramName = "frwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetIGain(paramDefault);

  paramName = "flwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetDGain(paramDefault);

  paramName = "frwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetDGain(paramDefault);

  this->UpdateHandWheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id = 0;
  this->dataPtr->flWheelRadius = this->CollisionRadius(
      this->dataPtr->flWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->frWheelRadius = this->CollisionRadius(
      this->dataPtr->frWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->blWheelRadius = this->CollisionRadius(
      this->dataPtr->blWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->brWheelRadius = this->CollisionRadius(
      this->dataPtr->brWheelJoint->GetChild()->GetCollision(id));

  // Get initial joint friction and add it to braking friction
  dPtr->flJointFriction = dPtr->flWheelJoint->GetParam("friction", 0);
  dPtr->frJointFriction = dPtr->frWheelJoint->GetParam("friction", 0);
  dPtr->blJointFriction = dPtr->blWheelJoint->GetParam("friction", 0);
  dPtr->brJointFriction = dPtr->brWheelJoint->GetParam("friction", 0);

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  ignition::math::Vector3d flCenterPos =
    this->dataPtr->flWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d frCenterPos =
    this->dataPtr->frWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d blCenterPos =
    this->dataPtr->blWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d brCenterPos =
    this->dataPtr->brWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();

  // track widths are computed first
  ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  this->dataPtr->frontTrackWidth = vec3.Length();
  vec3 = flCenterPos - frCenterPos;
  this->dataPtr->backTrackWidth = vec3.Length();
  // to compute wheelbase, first position of axle centers are computed
  ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  this->dataPtr->wheelbaseLength = vec3.Length();

  // gzerr << "wheel base length and track width: "
  //   << this->dataPtr->wheelbaseLength << " "
  //   << this->dataPtr->frontTrackWidth
  //   << " " << this->dataPtr->backTrackWidth << std::endl;

  // Max force that can be applied to hand steering wheel
  double handWheelForce = 10;
  this->dataPtr->handWheelPID.Init(100, 0, 10, 0, 0,
      handWheelForce, -handWheelForce);

  // Max force that can be applied to wheel steering joints
  double kMaxSteeringForceMagnitude = 5000;

  this->dataPtr->flWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->flWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  this->dataPtr->frWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->frWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PriusHybridPlugin::Update, this));

  this->dataPtr->keyboardSub =
    this->dataPtr->gznode->Subscribe("~/keyboard/keypress",
        &PriusHybridPlugin::OnKeyPress, this, true);

  this->dataPtr->worldControlPub =
    this->dataPtr->gznode->Advertise<msgs::WorldControl>("~/world_control");

  this->dataPtr->node.Subscribe("/keypress", &PriusHybridPlugin::OnKeyPressIgn,
      this);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnCmdVel(const ignition::msgs::Pose &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->gasPedalPercent = std::min(_msg.position().x(), 1.0);
  this->dataPtr->handWheelCmd = _msg.position().y();
  this->dataPtr->brakePedalPercent = _msg.position().z();

  this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
  this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
}
/////////////////////////////////////////////////
void PriusHybridPlugin::OnCmdGear(const ignition::msgs::Int32 &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // -1 reverse, 0 neutral, 1 forward
  int state = static_cast<int>(this->dataPtr->directionState);
  state += _msg.data();
  state = ignition::math::clamp(state, -1, 1);
  this->dataPtr->directionState =
      static_cast<PriusHybridPluginPrivate::DirectionType>(state);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnCmdMode(const ignition::msgs::Boolean &/*_msg*/)
{
  // toggle ev mode
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->evMode = !this->dataPtr->evMode;
}

/////////////////////////////////////////////////
void PriusHybridPlugin::KeyControlTypeA(const int _key)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  switch (_key)
  {
    // e - gas pedal
    case 69:
    case 101:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // w - release pedals
    case 87:
    case 119:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // q - brake
    case 113:
    {
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->brakePedalPercent += 0.1;
      this->dataPtr->brakePedalPercent =
          std::min(this->dataPtr->brakePedalPercent, 1.0);
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->dataPtr->handWheelCmd += 0.25;
      this->dataPtr->handWheelCmd = std::min(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelHigh);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->dataPtr->handWheelCmd -= 0.25;
      this->dataPtr->handWheelCmd = std::max(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelLow);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // s - center steering
    case 83:
    case 115:
    {
      this->dataPtr->handWheelCmd = 0;
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // z reverse
    case 90:
    case 122:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::REVERSE;
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    }
    // c forward
    case 67:
    case 99:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
      break;
    }

    default:
    {
      this->dataPtr->brakePedalPercent = 0;
      this->dataPtr->gasPedalPercent = 0;
      break;
    }
  }
}


/////////////////////////////////////////////////
void PriusHybridPlugin::KeyControlTypeB(const int _key)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  switch (_key)
  {
    // w - accelerate forward
    case 87:
    case 119:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // a - steer left
    case 65:
    case 97:
    {
      this->dataPtr->handWheelCmd += 0.25;
      this->dataPtr->handWheelCmd = std::min(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelHigh);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // s - reverse
    case 83:
    case 115:
    {
      this->dataPtr->brakePedalPercent = 0.0;
      if (this->dataPtr->directionState != PriusHybridPluginPrivate::REVERSE)
        this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->gasPedalPercent += 0.1;
      this->dataPtr->gasPedalPercent =
          std::min(this->dataPtr->gasPedalPercent, 1.0);
      this->dataPtr->directionState = PriusHybridPluginPrivate::REVERSE;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // d - steer right
    case 68:
    case 100:
    {
      this->dataPtr->handWheelCmd -= 0.25;
      this->dataPtr->handWheelCmd = std::max(this->dataPtr->handWheelCmd,
          this->dataPtr->handWheelLow);
      this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // e brake
    case 69:
    case 101:
    {
      this->dataPtr->brakePedalPercent = 1.0;
      this->dataPtr->gasPedalPercent = 0.0;
      this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
      break;
    }
    // x neutral
    case 88:
    case 120:
    {
      this->dataPtr->directionState = PriusHybridPluginPrivate::NEUTRAL;
      break;
    }
    // q - EV mode
    case 81:
    case 113:
    {
      // avoid rapid mode changes due to repeated key press
      common::Time now = this->dataPtr->world->SimTime();
      if ((now - this->dataPtr->lastModeCmdTime).Double() > 0.3)
      {
        this->dataPtr->evMode = !this->dataPtr->evMode;
        this->dataPtr->lastModeCmdTime = now;
      }
      break;
    }
    default:
    {
      break;
    }
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::KeyControl(const int _key)
{
  if (this->dataPtr->keyControl == 0)
    this->KeyControlTypeA(_key);
  else if (this->dataPtr->keyControl == 1)
    this->KeyControlTypeB(_key);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  this->KeyControl(_msg->int_value());
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnKeyPressIgn(const ignition::msgs::Any &_msg)
{
  this->KeyControl(_msg.int_value());
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnReset(const ignition::msgs::Any & /*_msg*/)
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(true);

  this->dataPtr->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void PriusHybridPlugin::OnStop(const ignition::msgs::Any & /*_msg*/)
{
  ignition::msgs::StringMsg req;
  ignition::msgs::StringMsg rep;
  bool result = false;
  unsigned int timeout = 5000;
  bool executed = this->dataPtr->node.Request("/priuscup/upload",
      req, timeout, rep, result);
  if (executed)
  {
    std::cerr << "Result: " << result << std::endl;
    std::cerr << rep.data() << std::endl;
  }
  else
  {
    std::cerr << "Service call timed out" << std::endl;
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Reset()
{
  this->dataPtr->odom = 0;
  this->dataPtr->flWheelSteeringPID.Reset();
  this->dataPtr->frWheelSteeringPID.Reset();
  this->dataPtr->handWheelPID.Reset();
  this->dataPtr->lastMsgTime = 0;
  this->dataPtr->lastSimTime = 0;
  this->dataPtr->lastModeCmdTime = 0;
  this->dataPtr->lastPedalCmdTime = 0;
  this->dataPtr->lastSteeringCmdTime = 0;
  this->dataPtr->directionState = PriusHybridPluginPrivate::FORWARD;
  this->dataPtr->flWheelSteeringCmd = 0;
  this->dataPtr->frWheelSteeringCmd = 0;
  this->dataPtr->handWheelCmd = 0;
  this->dataPtr->batteryCharge = 0.75;
  this->dataPtr->gasConsumption = 0;
  this->dataPtr->gasPedalPercent = 0;
  this->dataPtr->brakePedalPercent = 0;
  this->dataPtr->handbrakePercent = 1.0;
  this->dataPtr->handWheelAngle  = 0;
  this->dataPtr->flSteeringAngle = 0;
  this->dataPtr->frSteeringAngle = 0;
  this->dataPtr->flWheelAngularVelocity  = 0;
  this->dataPtr->frWheelAngularVelocity = 0;
  this->dataPtr->blWheelAngularVelocity = 0;
  this->dataPtr->brWheelAngularVelocity  = 0;
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Update()
{
  // shortcut to this->dataPtr
  PriusHybridPluginPrivate *dPtr = this->dataPtr.get();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  common::Time curTime = this->dataPtr->world->SimTime();
  double dt = (curTime - this->dataPtr->lastSimTime).Double();
  if (dt < 0)
  {
    this->Reset();
    return;
  }
  else if (ignition::math::equal(dt, 0.0))
  {
    return;
  }

  dPtr->handWheelAngle = dPtr->handWheelJoint->Position();
  dPtr->flSteeringAngle = dPtr->flWheelSteeringJoint->Position();
  dPtr->frSteeringAngle = dPtr->frWheelSteeringJoint->Position();

  dPtr->flWheelAngularVelocity = dPtr->flWheelJoint->GetVelocity(0);
  dPtr->frWheelAngularVelocity = dPtr->frWheelJoint->GetVelocity(0);
  dPtr->blWheelAngularVelocity = dPtr->blWheelJoint->GetVelocity(0);
  dPtr->brWheelAngularVelocity = dPtr->brWheelJoint->GetVelocity(0);

  dPtr->chassisLinearVelocity = dPtr->chassisLink->WorldCoGLinearVel();
  // Convert meter/sec to miles/hour
  double linearVel = dPtr->chassisLinearVelocity.Length() * 2.23694;

  // Distance traveled in miles.
  this->dataPtr->odom += (fabs(linearVel) * dt/3600.0);

  bool neutral = dPtr->directionState == PriusHybridPluginPrivate::NEUTRAL;

  this->dataPtr->lastSimTime = curTime;

  // Aero-dynamic drag on chassis
  // F: force in world frame, applied at center of mass
  // V: velocity in world frame of chassis center of mass
  // C: drag coefficient based on straight-ahead driving [N / (m/s)^2]
  // |V|: speed
  // V_hat: velocity unit vector
  // F = -C |V|^2 V_hat
  auto dragForce = -dPtr->chassisAeroForceGain *
        dPtr->chassisLinearVelocity.SquaredLength() *
        dPtr->chassisLinearVelocity.Normalized();
  dPtr->chassisLink->AddForce(dragForce);

  // PID (position) steering
  this->dataPtr->handWheelCmd =
    ignition::math::clamp(this->dataPtr->handWheelCmd,
        -this->dataPtr->maxSteer / this->dataPtr->steeringRatio,
        this->dataPtr->maxSteer / this->dataPtr->steeringRatio);
  double steerError =
      this->dataPtr->handWheelAngle - this->dataPtr->handWheelCmd;
  double steerCmd = this->dataPtr->handWheelPID.Update(steerError, dt);
  this->dataPtr->handWheelJoint->SetForce(0, steerCmd);
  //this->dataPtr->handWheelJoint->SetPosition(0, this->dataPtr->handWheelCmd);
  //this->dataPtr->handWheelJoint->SetLowStop(0, this->dataPtr->handWheelCmd);
  //this->dataPtr->handWheelJoint->SetHighStop(0, this->dataPtr->handWheelCmd);

  // PID (position) steering joints based on steering position
  // Ackermann steering geometry here
  //  \TODO provide documentation for these equations
  double tanSteer =
      tan(this->dataPtr->handWheelCmd * this->dataPtr->steeringRatio);
  this->dataPtr->flWheelSteeringCmd = atan2(tanSteer,
      1 - this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  this->dataPtr->frWheelSteeringCmd = atan2(tanSteer,
      1 + this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  // this->flWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;
  // this->frWheelSteeringCmd = this->handWheelAngle * this->steeringRatio;

  double flwsError =
      this->dataPtr->flSteeringAngle - this->dataPtr->flWheelSteeringCmd;
  double flwsCmd = this->dataPtr->flWheelSteeringPID.Update(flwsError, dt);
  this->dataPtr->flWheelSteeringJoint->SetForce(0, flwsCmd);
  // this->dataPtr->flWheelSteeringJoint->SetPosition(0,
  // this->dataPtr->flWheelSteeringCmd);
  // this->dataPtr->flWheelSteeringJoint->SetLowStop(0,
  // this->dataPtr->flWheelSteeringCmd);
  // this->dataPtr->flWheelSteeringJoint->SetHighStop(0,
  // this->dataPtr->flWheelSteeringCmd);

  double frwsError =
      this->dataPtr->frSteeringAngle - this->dataPtr->frWheelSteeringCmd;
  double frwsCmd = this->dataPtr->frWheelSteeringPID.Update(frwsError, dt);
  this->dataPtr->frWheelSteeringJoint->SetForce(0, frwsCmd);
  // this->dataPtr->frWheelSteeringJoint->SetPosition(0,
  // this->dataPtr->frWheelSteeringCmd);
  // this->dataPtr->frWheelSteeringJoint->SetLowStop(0,
  // this->dataPtr->frWheelSteeringCmd);
  // this->dataPtr->frWheelSteeringJoint->SetHighStop(0,
  // this->dataPtr->frWheelSteeringCmd);

  //static common::Time lastErrorPrintTime = 0.0;
  //if (curTime - lastErrorPrintTime > 0.01 || curTime < lastErrorPrintTime)
  //{
  //  lastErrorPrintTime = curTime;
  //  double maxSteerError =
  //    std::abs(frwsError) > std::abs(flwsError) ? frwsError : flwsError;
  //  double maxSteerErrPer = maxSteerError / this->dataPtr->maxSteer * 100.0;
  //  std::cerr << std::fixed << "Max steering error: " << maxSteerErrPer
  //    << std::endl;
  //}

  // Model low-speed caaaareep and high-speed regen braking
  // with term added to gas/brake
  // Cross-over speed is 7 miles/hour
  // 10% throttle at 0 speed
  // max 2.5% braking at higher speeds
  double creepPercent;
  if (std::abs(linearVel) <= 7)
  {
    creepPercent = 0.1 * (1 - std::abs(linearVel) / 7);
  }
  else
  {
    creepPercent = 0.025 * (7 - std::abs(linearVel));
  }
  creepPercent = ignition::math::clamp(creepPercent, -0.025, 0.1);

  // Gas pedal torque.
  // Map gas torques to individual wheels.
  // Cut off gas torque at a given wheel if max speed is exceeded.
  // Use directionState to determine direction of that can be applied torque.
  // Note that definition of DirectionType allows multiplication to determine
  // torque direction.
  // also, make sure gas pedal is at least as large as the creepPercent.
  double gasPercent = std::max(this->dataPtr->gasPedalPercent, creepPercent);
  double gasMultiplier = this->GasTorqueMultiplier();
  double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
  // Apply equal torque at left and right wheels, which is an implicit model
  // of the differential.
  if (fabs(dPtr->flWheelAngularVelocity * dPtr->flWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->frWheelAngularVelocity * dPtr->frWheelRadius) < dPtr->maxSpeed)
  {
    flGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
    frGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
  }
  if (fabs(dPtr->blWheelAngularVelocity * dPtr->blWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->brWheelAngularVelocity * dPtr->brWheelRadius) < dPtr->maxSpeed)
  {
    blGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
    brGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
  }
  double throttlePower =
      std::abs(flGasTorque * dPtr->flWheelAngularVelocity) +
      std::abs(frGasTorque * dPtr->frWheelAngularVelocity) +
      std::abs(blGasTorque * dPtr->blWheelAngularVelocity) +
      std::abs(brGasTorque * dPtr->brWheelAngularVelocity);

  // auto release handbrake as soon as the gas pedal is depressed
  if (this->dataPtr->gasPedalPercent > 0)
    this->dataPtr->handbrakePercent = 0.0;

  double brakePercent = this->dataPtr->brakePedalPercent
      + this->dataPtr->handbrakePercent;
  // use creep braking if not in Neutral
  if (!neutral)
  {
    brakePercent = std::max(brakePercent,
        -creepPercent - this->dataPtr->gasPedalPercent);
  }

  brakePercent = ignition::math::clamp(brakePercent, 0.0, 1.0);
  dPtr->flWheelJoint->SetParam("friction", 0,
      dPtr->flJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->frWheelJoint->SetParam("friction", 0,
      dPtr->frJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->blWheelJoint->SetParam("friction", 0,
      dPtr->blJointFriction + brakePercent * dPtr->backBrakeTorque);
  dPtr->brWheelJoint->SetParam("friction", 0,
      dPtr->brJointFriction + brakePercent * dPtr->backBrakeTorque);

  this->dataPtr->flWheelJoint->SetForce(0, flGasTorque);
  this->dataPtr->frWheelJoint->SetForce(0, frGasTorque);
  this->dataPtr->blWheelJoint->SetForce(0, blGasTorque);
  this->dataPtr->brWheelJoint->SetForce(0, brGasTorque);

  // gzerr << "gas and brake torque " << flGasTorque << " "
  //       << flBrakeTorque << std::endl;

  // Battery

  // Speed x throttle regions
  //
  //    throttle |
  //             |
  //        high |____
  //             |    |
  //      medium |____|_____
  //             |    |     |
  //         low |____|_____|_________
  //              low  med   high    speed

  bool engineOn;
  bool regen = !neutral;
  double batteryChargePower = 0;
  double batteryDischargePower = 0;

  // Battery is below threshold
  if (this->dataPtr->batteryCharge < this->dataPtr->batteryLowThreshold)
  {
    // Gas engine is on and recharing battery
    engineOn = true;
    this->dataPtr->evMode = false;
    batteryChargePower = dPtr->kLowBatteryChargePower;
    throttlePower += dPtr->kLowBatteryChargePower;
  }
  // Neutral and battery not low
  else if (neutral)
  {
    // Gas engine is off, battery not recharged
    engineOn = false;
  }
  // Speed below medium-high threshold, throttle below low-medium threshold
  else if (linearVel < this->dataPtr->speedMediumHigh &&
      this->dataPtr->gasPedalPercent <= this->dataPtr->kGasPedalLowMedium)
  {
    // Gas engine is off, running on battery
    engineOn = false;
    batteryDischargePower = throttlePower;
  }
  // EV mode, speed below low-medium threshold, throttle below medium-high
  // threshold
  else if (this->dataPtr->evMode && linearVel < this->dataPtr->speedLowMedium
      && this->dataPtr->gasPedalPercent <= this->dataPtr->kGasPedalMediumHigh)
  {
    // Gas engine is off, running on battery
    engineOn = false;
    batteryDischargePower = throttlePower;
  }
  else
  {
    // Gas engine is on
    engineOn = true;
    this->dataPtr->evMode = false;
  }

  if (regen)
  {
    // regen max torque at same level as throttle limit in EV mode
    // but only at the front wheels
    batteryChargePower +=
      std::min(this->dataPtr->kGasPedalMediumHigh, brakePercent)*(
        dPtr->frontBrakeTorque * std::abs(dPtr->flWheelAngularVelocity) +
        dPtr->frontBrakeTorque * std::abs(dPtr->frWheelAngularVelocity) +
        dPtr->backBrakeTorque * std::abs(dPtr->blWheelAngularVelocity) +
        dPtr->backBrakeTorque * std::abs(dPtr->brWheelAngularVelocity));
  }
  dPtr->batteryCharge += dt / 3600 * (
      batteryChargePower / dPtr->batteryChargeWattHours
    - batteryDischargePower / dPtr->batteryDischargeWattHours);
  if (dPtr->batteryCharge > 1)
  {
    dPtr->batteryCharge = 1;
  }

  // engine has minimum gas flow if the throttle is pressed at all
  if (engineOn && throttlePower > 0)
  {
    dPtr->gasConsumption += dt*(dPtr->minGasFlow
        + throttlePower / dPtr->gasEfficiency / dPtr->kGasEnergyDensity);
  }

  // Accumulated mpg since last reset
  // max value: 999.9
  double mpg = std::min(999.9,
      dPtr->odom / std::max(dPtr->gasConsumption, 1e-6));

  if ((curTime - this->dataPtr->lastMsgTime) > .5)
  {
    this->dataPtr->posePub.Publish(
        ignition::msgs::Convert(this->dataPtr->model->WorldPose()));

    ignition::msgs::Double_V consoleMsg;

    // linearVel (meter/sec) = (2*PI*r) * (rad/sec).
    double linearVel = (2.0 * IGN_PI * this->dataPtr->flWheelRadius) *
      ((this->dataPtr->flWheelAngularVelocity +
        this->dataPtr->frWheelAngularVelocity) * 0.5);

    // Convert meter/sec to miles/hour
    linearVel *= 2.23694;

    // Distance traveled in miles.
    this->dataPtr->odom += (fabs(linearVel) * dt/3600);

    // \todo: Actually compute MPG
    double mpg = 1.0 / std::max(linearVel, 0.0);

    // Gear information: 1=drive, 2=reverse, 3=neutral
    if (this->dataPtr->directionState == PriusHybridPluginPrivate::FORWARD)
      consoleMsg.add_data(1.0);
    else if (this->dataPtr->directionState == PriusHybridPluginPrivate::REVERSE)
      consoleMsg.add_data(2.0);
    else if (this->dataPtr->directionState == PriusHybridPluginPrivate::NEUTRAL)
      consoleMsg.add_data(3.0);

    // MPH. A speedometer does not go negative.
    consoleMsg.add_data(std::max(linearVel, 0.0));

    // MPG
    consoleMsg.add_data(mpg);

    // Miles
    consoleMsg.add_data(this->dataPtr->odom);

    // EV mode
    this->dataPtr->evMode ? consoleMsg.add_data(1.0) : consoleMsg.add_data(0.0);

    // Battery state
    consoleMsg.add_data(this->dataPtr->batteryCharge);

    this->dataPtr->consolePub.Publish(consoleMsg);

    // Output prius car data.
    this->dataPtr->posePub.Publish(
        ignition::msgs::Convert(this->dataPtr->model->WorldPose()));

    this->dataPtr->lastMsgTime = curTime;
  }

  // reset if last command is more than x sec ago
  if ((curTime - this->dataPtr->lastPedalCmdTime).Double() > 0.3)
  {
    this->dataPtr->gasPedalPercent = 0.0;
    this->dataPtr->brakePedalPercent = 0.0;
  }

  if ((curTime - this->dataPtr->lastSteeringCmdTime).Double() > 0.3)
  {
    this->dataPtr->handWheelCmd = 0;
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::UpdateHandWheelRatio()
{
  // The total range the steering wheel can rotate
  this->dataPtr->handWheelHigh = 7.85;
  this->dataPtr->handWheelLow = -7.85;
  double handWheelRange =
      this->dataPtr->handWheelHigh - this->dataPtr->handWheelLow;
  double high = 0.8727;
  high = std::min(high, this->dataPtr->maxSteer);
  double low = -0.8727;
  low = std::max(low, -this->dataPtr->maxSteer);
  double tireAngleRange = high - low;

  // Compute the angle ratio between the steering wheel and the tires
  this->dataPtr->steeringRatio = tireAngleRange / handWheelRange;
}

/////////////////////////////////////////////////
// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double PriusHybridPlugin::CollisionRadius(physics::CollisionPtr _coll)
{
  if (!_coll || !(_coll->GetShape()))
    return 0;
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(_coll->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

/////////////////////////////////////////////////
double PriusHybridPlugin::GasTorqueMultiplier()
{
  // if (this->dataPtr->keyState == ON)
  {
    if (this->dataPtr->directionState == PriusHybridPluginPrivate::FORWARD)
      return 1.0;
    else if (this->dataPtr->directionState == PriusHybridPluginPrivate::REVERSE)
      return -1.0;
  }
  return 0;
}

GZ_REGISTER_MODEL_PLUGIN(PriusHybridPlugin)
```



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

