# Raspimouse Node

ROS 2 node for the Raspimouse robot from RT.

https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3419

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


## Known problems

- Due to instabilities in either the hardware pulse counters or the kernel driver for them, it is
  possible for reading from the pulse counters to cause the node to freeze. If this happens
  regularly, disable the hardware pulse counters using the `use_pulse_counters` parameter and
  rely on estimated odometry instead.

## Contributors

* Geoffrey Biggs ([@gbiggs](https://github.com/gbiggs)), **original author**
* Shota Hirama ([@shotahirama](https://github.com/shotahirama))
* Yutaka Kondo ([@youtalk](https://github.com/youtalk))

Contributions are always welcome!
