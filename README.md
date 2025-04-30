# mav1500_base_control

Starting the driver of the base:

```
ros2 launch diff_canopen_system mav1500_drive_system.launch.xml
```

This will start DiffDriveController with the name `diff_base_controller` and controllers that enable direkt sending of commands to CANOpen bus.
Those might be useful for debugging.

### Send command to the controller

```
ros2 topic pub -r 10 /diff_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
 stamp:
  sec: 0
  nanosec: 0
 frame_id: ''
twist:
 linear:
  x: 0.3
  y: 0.0
  z: 0.0
 angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

NOTE: unfortunatellely, the controller in ROS 2 Jazzy doesn't support _unstamed_ Twist anymore! Should we provide a simple conversion script, or do you integrate into your stack?

### Getting data from the controller

To get odometry subscribe to:

```
/diff_base_controller/odom
```

There is also TF from `odom` to `base_link` published on the global `/tf` topic.

the velocity and toruqe (effort) you also get on the `/joint_states` topic.

### Adjusting controller parameters

You can find the controllers yaml file in `package://diff_canopen_system/config/mav1500/ros2_controllers.yaml` (for now - it will be moved to dedicated package). Currently controllers are running on 50 Hz.

### Accessing the other values from the motor controllers

For now use `/dynamic_joint_states` topic to access all available data from the robot.
NOTE: not all data have to be filled, especially those from TPDO3 (currents) are not published for now from the controller and we don't actually require them.

###### Observing the battery

- Use `battery_voltage` field from `/dynamic_joint_states`
- `BDI_level` should deliver procentage level of the battery (1-10), but it seems to be "stuck" on 10
- There are additionall encoder data (ticks): `encoder_pulse_counter` if required
- Temperature of the motor and inverter (if relevant)

### Tips and Tricks

#####

Stopping `Navitrol` program. The best is to use the following commands:

```
ps -e | grep Nav   # you should get two process
kill -9 <pid>      # pid you get from `ps` command
kill -9 <second_pid>
```

If you do this properly, then the robot's dispaly will still work.

##### can network

It is recommended to `up/down` `can0` interface before starting:

```
ip link set can0 down
ip link set can0 up
```

##### safety stops

If you enter to the laser scanners the robot will stop, and automatically will be reinitialized afterward (and moving!!) - please handle this cases in the upper-layered software!

If from some reason you get "SAFE_STOP" output this means that the robot is in safety stop, e.g., by pressing emergency stop buttons. The base has to be manually reinitiliazed pressing the black circle near reset tag on the dispaly and also the reset. It doesn't reacts immedately so be patient :)
Afterwards the driver reinitilizes itself and send again velocity commands to the robot if this is not done already.

##### error messages

`Controller 'right_wheel' (0x1F): no new data (bit is not toggeled).` - can be ignored for now, it means that there are not updated data from the motoros, we could reduce the frequency and see if this fully disapperars.

### Things to do

- Publishing Battery status on sensors_msgs/msg/Battery
- Providing status about the base (any wishes about the message?)
- Cleaning up the driver and mybe writing dedicated controller

### Using docker on the robot

Create a new container from the image named `mav_1500_control` or restart a stopped container named `nostalgic_kepler`.

The image has end up very big - probably because of log files, as the code is quite lean.

Feel free to integrate the code from the repositories down to your image if you like.
Everything is defined correctly and `rosdep` will do the work with dependencies.

**As soon as you enter the Docker, ROS 2 workspace is sourced.**

### Repositories

```
repositories:
  diff_canopen_system:
    type: git
    url: https://github.com/b-robotized/diff_canopen_system.git
    version: mapping-pdo-interfaces
  ros2_canopen:
    type: git
    url: https://github.com/ros-industrial/ros2_canopen
    version: fixed-type-handling
```

## Testing with fake diff drive system

### Setup the vcan0 interface

```

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0

```
