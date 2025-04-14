# raspimosue2


[![industrial_ci](https://github.com/rt-net/raspimouse2/actions/workflows/industrial_ci.yml/badge.svg?branch=master)](https://github.com/rt-net/raspimouse2/actions/workflows/industrial_ci.yml)

![raspimouse](https://rt-net.github.io/images/raspberry-pi-mouse/Raspberry-Pi-Mouse.png)

ROS 2 node for the Raspberry Pi Mouse.

**This branch is dedicated to ROS 2 Jazzy. For other distributions, please refer to the corresponding branches listed below.**
- ROS 2 Humble ([humble](https://github.com/rt-net/raspimouse2/tree/humble?tab=readme-ov-file))

## Table of Contents

<!-- 必須 -->

- [raspimosue2](#raspimosue2)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS distributions](#supported-ros-distributions)
    - [ROS 2](#ros-2)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Binary Installation](#binary-installation)
    - [Source Build](#source-build)
  - [QuickStart](#quickstart)
  - [Packages](#packages)
  - [Topics](#topics)
    - [Subscribed](#subscribed)
    - [Published](#published)
  - [Services](#services)
  - [Parameters](#parameters)
  - [Node Description](#node-description)
  - [License](#license)
  - [Contributing](#contributing)
  - [Contributors](#contributors)

## Supported ROS distributions

### ROS 2

- [Humble Hawksbill](https://github.com/rt-net/raspimouse2/tree/humble)
- [Jazzy Jalisco](https://github.com/rt-net/raspimouse2/tree/jazzy)
- 
## Requirements

- Raspberry Pi Mouse
  - [Summary](https://rt-net.jp/products/raspberrypimousev3/)
  - [RT Robot Shop](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=4141)
- Linux OS
  - Ubuntu server
    - 22.04
    - 24.04
- Device Driver
  - [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)
- ROS 2
  - [Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
  - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## Installation

### Binary Installation

```sh
sudo apt install ros-$ROS_DISTRO-raspimouse
```


### Source Build

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/

# Clone package
git clone -b $ROS_DISTRO https://github.com/rt-net/raspimouse2

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## QuickStart

Build and install the [device driver](https://github.com/rt-net/RaspberryPiMouse) in advance.

```sh
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch raspimouse raspimouse.launch.py

# Terminal 2
source ~/ros2_ws/install/setup.bash
# Set buzzer frequency
ros2 topic pub -1 /buzzer std_msgs/msg/Int16 '{data: 1000}'
ros2 topic pub -1 /buzzer std_msgs/msg/Int16 '{data: 0}'
# or rotate motors
ros2 service call /motor_power std_srvs/SetBool '{data: true}'
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.05, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.05}}}'
# Shutdown
ros2 lifecycle set raspimouse shutdown
```

## Packages

- raspimouse

  This package controls the Raspberry Pi Mouse using the device driver.

- raspimouse_msgs

  This package defines the custom message types used by the Raspberry Pi Mouse.

## Topics 

### Subscribed 

- `buzzer`

  Type: `std_msgs/Int16`

  Used to control the buzzer. Provide a value in Hertz and the buzzer will emit that tone.

- `cmd_vel`

  Type: `geometry_msgs/msg/TwistStamped`

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

## Node Description

This is a managed-lifecycle node. The node must be configured and activated after being launched
before the robot can be used. If running the node manually, after launching the node execute the
following command to configure it:

```shell
ros2 lifecycle set raspimouse configure
```

If configuration succeeds, execute the following command to activate the node:

```shell
ros2 lifecycle set raspimouse activate
```

The robot can now be controlled and sensor information will be published.

The node can be deactivated using the following command:

```shell
ros2 lifecycle set raspimouse deactivate
```

This will disable publishing sensor data and stop the motors.

When the node is active, motor functionality can be tested by turning on the motors and sending
a velocity command.

```shell
ros2 service call /motor_power std_srvs/SetBool '{data: true}'
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.05}}}'
```

Odometry information can be checked by echoing the `odom` topic.

```shell
ros2 topic echo /odom
```

Similarly other sensor information can also be viewed by echoing the relevant topic.

## License

This repository is licensed under the Apache 2.0, see [LICENSE](./LICENSE) for details.

## Contributing

- This software is open source, but its development is not open.
- This software is essentially provided as open source software on an “AS IS” (in its current state) basis.
- No free support is available for this software.
- Requests for bug fixes and corrections of typographical errors are always accepted; however, requests for additional features will be subject to our internal guidelines. For further details, please refer to the [Contribution Guidelines]((https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md)).

## Contributors

* Geoffrey Biggs ([@gbiggs](https://github.com/gbiggs)), **original author**
* Shota Hirama ([@shotahirama](https://github.com/shotahirama))
* Yutaka Kondo ([@youtalk](https://github.com/youtalk))

Contributions are always welcome!

