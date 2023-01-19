# Raspimouse Node

ROS 2 node for the Raspberry Pi Mouse.

![raspimouse](https://rt-net.jp/wp-content/uploads/2020/04/Raspberry-Pi-Mouse.png)

## Build Status

### master branch

[![industrial_ci](https://github.com/rt-net/raspimouse2/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse2/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

### Source Build Status on ROS2 Buildfarm

| ROS 2 + Ubuntu | raspimouse | raspimouse_msgs |
|:---:|:---:|:---:|
| Foxy + Focal ([`foxy-devel`](https://github.com/rt-net/raspimouse2/tree/foxy-devel)) | [![Build Status](https://build.ros2.org/view/Fsrc_uF/job/Fsrc_uF__raspimouse__ubuntu_focal__source/badge/icon)](https://build.ros2.org/view/Fsrc_uF/job/Fsrc_uF__raspimouse__ubuntu_focal__source/) | [![Build Status](https://build.ros2.org/view/Fsrc_uF/job/Fsrc_uF__raspimouse_msgs__ubuntu_focal__source/badge/icon)](https://build.ros2.org/view/Fsrc_uF/job/Fsrc_uF__raspimouse_msgs__ubuntu_focal__source/) |
| Humble + Jammy ([`humble-devel`](https://github.com/rt-net/raspimouse2/tree/humble-devel)) | **TODO** | **TODO** |

## Requirements

- Raspberry Pi Mouse
  - https://rt-net.jp/products/raspberrypimousev3/
  - [RT Robot Shop](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1395&products_id=3774)
- Linux OS
  - Ubuntu server 20.04
  - https://ubuntu.com/download/raspberry-pi
- Device Driver
  - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
- ROS
  - [Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)
  - [Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## Installation

### Binary Insallation

```sh
$ sudo apt install ros-$ROS_DISTRO-raspimouse
```

### Source Build

```sh
$ cd ~/ros2_ws/src
# Clone package
$ git clone -b $ROS_DISTRO-devel https://github.com/rt-net/raspimouse2

# Install dependencies
$ rosdep install -r -y -i --from-paths .

# Build & Install
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## QuickStart

```sh
# Terminal 1
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch raspimouse raspimouse.launch.py

# Terminal 2
$ source ~/ros2_ws/install/setup.bash
# Set buzzer frequency
$ ros2 topic pub -1 /buzzer std_msgs/msg/Int16 '{data: 1000}'
$ ros2 topic pub -1 /buzzer std_msgs/msg/Int16 '{data: 0}'
# or rotate motors
$ ros2 service call /motor_power std_srvs/SetBool '{data: true}'
$ ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.05}}'
# Shutdown
$ ros2 lifecycle set raspimouse shutdown
```

## Node Description

This is a managed-lifecycle node. The node must be configured and activated after being launched
before the robot can be used. If running the node manually, after launching the node execute the
following command to configure it:

```shell
$ ros2 lifecycle set raspimouse configure
```

If configuration succeeds, execute the following command to activate the node:

```shell
$ ros2 lifecycle set raspimouse activate
```

The robot can now be controlled and sensor information will be published.

The node can be deactivated using the following command:

```shell
$ ros2 lifecycle set raspimouse deactivate
```

This will disable publishing sensor data and stop the motors.

When the node is active, motor functionality can be tested by turning on the motors and sending
a velocity command.

```shell
$ ros2 service call /motor_power std_srvs/SetBool '{data: true}'
$ ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.05}}'
```

Odometry information can be checked by echoing the `odom` topic.

```shell
$ ros2 topic echo /odom
```

Similarly other sensor information can also be viewed by echoing the relevant topic.


## Topics

### Subscribed

- `buzzer`

  Type: `std_msgs/Int16`

  Used to control the buzzer. Provide a value in Hertz and the buzzer will emit that tone.

- `cmd_vel`

  Type: `geometry_msgs/Twist`

  Controls the motors. Specify the forward and turning speeds of the robot.

- `leds`

  Type: `raspimouse_msgs/Leds`

  Turns the four LEDs on the front of the robot on and off.


### Published

- `light_sensors`

  Type: `raspimouse_msgs/LightSensors`

  Provides the raw values from the light sensors on the front of the robot.

- `odom`

  Type: `nav_msgs/Odometry`

  Provides odometry. If hardware pulse counters are available and the `use_pulse_counters`
  parameter is set to `true`, the odometry is calculated from the motor control pulse counts.
  Otherwise, the odometry is estimated based on the velocity commands given to the robot and
  elapsed time.

- `switches`

  Type: `raspimouse_msgs/Switches`

  Provides the status of each of the three push switches on the side of the robot.


## Services

- `motor_power`

  Type: `std_srvs/SetBool`

  Call this service and pass `true` to enable the motors. Pass `false` to disable the motors.


## Parameters

- `odometry_scale_left_wheel`

  Type: `double`

  Default: `1.0`

  Use to adjust the odometry input from the left wheel (when using pulse counters to calculate
  odometry). This is used to account for slight differences in wheel diameter and wheel slip
  between the left and right wheels.

- `odometry_scale_right_wheel`

  Type: `double`

  Default: `1.0`

  Use to adjust the odometry input from the right wheel (when using pulse counters to calculate
  odometry). This is used to account for slight differences in wheel diameter and wheel slip
  between the left and right wheels.

- `use_light_sensors`

  Type: `boolean`

  Default: `true`

  Enable or disable the light sensors on the front of the robot.

- `use_pulse_counters`

  Type: `boolean`

  Default: `false`

  Use hardware pulse counters as the odometry source. When set to true, hardware pulse counters
  will be used only if present.

- `wheel_diameter`

  Type: `double`

  Default: `0.048`

  Sets the diameter of the robot's wheel.
  The unit is in meters.
  
- `wheel_tread`

  Type: `double`

  Default: `0.0925`

  Sets the distance between the wheels.
  The unit is in meters.

- `pulses_per_revolution`

  Type: `double`

  Default: `400.0`

  Sets the number of pulses needed for 1 rotation of the used motor.

- `light_sensors_hz`

  Type: `double`

  Default: `100.0`

  Sets the frequency of the publishing rate of the topic `light_sensors`.
  The unit is in Hz.

- `odom_hz`

  Type: `double`

  Default: `100.0`

  Sets the frequency of the publishing rate of the topic `odom`.
  The unit is in Hz.

- `switches_hz`

  Type: `double`

  Default: `10.0`

  Sets the frequency of the publishing rate of the topic `switches`.
  The unit is in Hz.

- `initial_motor_power`

  Type: `bool`

  Default: `False`

  Sets the initial state of the motor.
  If set as `True`, the motors will turn on when the `raspimouse` node becomes active.

- `odom_frame_id`

  Type: `string`

  Default: `odom`

  Sets the frame_id of the topic `odom`.

- `odom_child_frame_id`

  Type: `string`

  Default: `base_footprint`

  Sets the child_frame_id of the topic `odom`.

- `odom_frame_prefix`

  Type: `string`

  Default: `{empty}`

  Adds prefix to the frames of the topic `odom`.
  If set as *`mouse`*, the frame_id and the child_frame_id will be *`mouse/odom`* and *`mouse/baes_footprint`*.
  
## License

This repository is licensed under the Apache 2.0, see [LICENSE](./LICENSE) for details.

## Contributors

* Geoffrey Biggs ([@gbiggs](https://github.com/gbiggs)), **original author**
* Shota Hirama ([@shotahirama](https://github.com/shotahirama))
* Yutaka Kondo ([@youtalk](https://github.com/youtalk))

Contributions are always welcome!
