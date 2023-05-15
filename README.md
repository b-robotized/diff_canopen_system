# diff_canopen_system

How to develop a CiA301 profile yourself:

There are two component you have to take care of: hardware_interface and controller.
https://github.com/StoglRobotics/diff_canopen_system/blob/master/src/DiffCanopenSystem.cpp

For hardware_interface, we will extend a child class from the CanOpenSystem (ToDo: check the name). The important part here is to prepare the following functions:
- export_state_interfaces: register the state you want to read from the hardware and associate the variables to them
- export_command_interfaces: register the command you want to sent to the hardware
- read(): read from the hardware or request the hardware to send back the state (via RPDO)
- write(): write command to the hardware (via TPDO)

Configuration:
- plugin export xml: define the name(for including) and the type(for using) of this hardware interface


For the contollers, you should extend the controller AND provide with proper configuration and urdf.
The controller can be a child class of canopen_ros2_controllers::CanopenProxyController. Following function should be filled:
- command_interface_configuration
- state_interface_configuration


Configuration: (ToDo: Do we need example for this?)
- urdf
    - *.urdf.xacro
        - Include configuration for bus, master and can interface for ros2_control
        - Define the link and joints as usual
    - *.ros_control.xacro
        - Claim the hardware using the parameter defined in the urdf.xacro
        - Assign the controllers proper can node_id
- ros2_control config
    - controller manager: define cantroller names and their types
    - controller: define parameters you need to initialize the controllers
    - bus: define master and slaves
        - master: node id, driver type, package
        - slaves: node id, dcf, driver and packages
- can interface: define command and state interface

---
(ToDo)
How does the controller connect with the hardware interface? How are the states and the commands transferred?

We have registered the command and state interfaces in the hardware interface. By claiming the same resources in the controller, the controller is able to read data and write command. We should be aware of the indices we use.

The hardware interface connects to the can device driver. While the hardware interface gets the new states from the hardware, the state interface will update its states at the same time. The controller registered to the same state interface has access to the new states as well. Similarly, as the controller set a target command, the hardware interface can get this updated command via the registered command interface and forward the command to hardware.

https://github.com/ros-industrial/ros2_canopen/blob/master/canopen_ros2_controllers/src/canopen_proxy_controller.cpp


https://github.com/StoglRobotics-forks/ros2_canopen/blob/master-galactic-backport/canopen_ros2_controllers/src/cia402_device_controller.cpp
 
