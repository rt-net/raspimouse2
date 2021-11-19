# Raspimouse Node

ROS 2 node for the Raspberry Pi Mouse.

![raspimouse](https://rt-net.jp/wp-content/uploads/2020/04/Raspberry-Pi-Mouse.png)

## Build Status

### master branch

[![industrial_ci](https://github.com/rt-net/raspimouse2/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/raspimouse2/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

### Source Build Status on ROS2 Buildfarm
#### Bionic + Dashing ([`dashing-devel`](https://github.com/rt-net/raspimouse2/tree/dashing-devel))

| raspimouse | raspimouse_msgs |
|:---:|:---:|
| [![](http://build.ros2.org/buildStatus/icon?job=Dsrc_uB__raspimouse__ubuntu_bionic__source)](http://build.ros2.org/view/Dsrc_uB/job/Dsrc_uB__raspimouse__ubuntu_bionic__source/) | [![](http://build.ros2.org/buildStatus/icon?job=Dsrc_uB__raspimouse_msgs__ubuntu_bionic__source/)](http://build.ros2.org/view/Dsrc_uB/job/Dsrc_uB__raspimouse_msgs__ubuntu_bionic__source/) |

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
$ ros2 run raspimouse raspimouse


# Terminal 2
$ source ~/ros2_ws/install/setup.bash
$ ros2 lifecycle set raspimouse configure

# Set buzzer frequency
$ ros2 topic pub -1 /buzzer std_msgs/msg/Int16 '{data: 1000}'
$ ros2 topic pub -1 /buzzer std_msgs/msg/Int16 '{data: 0}'

# or rotate motors
$ ros2 lifecycle set raspimouse activate
$ ros2 service call /motor_power std_srvs/SetBool '{data: true}'
$ ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.05, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.05}}'
$ ros2 lifecycle set raspimouse deactivate
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
$ ros2 topic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.05, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.05}}'
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


## License

This repository is licensed under the Apache 2.0, see [LICENSE](./LICENSE) for details.

## Contributors

* Geoffrey Biggs ([@gbiggs](https://github.com/gbiggs)), **original author**
* Shota Hirama ([@shotahirama](https://github.com/shotahirama))
* Yutaka Kondo ([@youtalk](https://github.com/youtalk))

Contributions are always welcome!
