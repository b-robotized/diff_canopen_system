# diff_canopen_system

To bring up the system, we need to be aware of these things
1. Currently, we prepared the request message to get the motor status. We have to put the proper index and data to request for the states. Look up for
these functions:
- send_motor_status_request
- send_error_status_request
- send_motor_battery_request

2. After we sent the request, the could data via RPDO. However, the recieved data is not processed. For example, after we get the data from motor status,
we have to split them into RPM, power and temperature and write them to the state interface. 
Currently, the code recieves the data, but put only dummy numbers into the state interface after the recieval.
Look up the following functions to make the convertion complete:
- read_motor_status
- read_error_status
- read_motor_battery_states

How to develop a CiA301 profile yourself:

There are two component you have to take care of: hardware_interface and controller.
https://github.com/StoglRobotics/diff_canopen_system/blob/master/src/DiffCanopenSystem.cpp

For hardware_interface, we will extend a child class from the CanOpenSystem (ToDo: check the name). The important part here is to prepare the following functions:
- export_state_interfaces: register the state you want to read from the hardware and associate the variables to them
- export_command_interfaces: register the command you want to sent to the hardware
- read(): read from the hardware or request the hardware to send back the state (via RPDO)
- write(): write command to the hardware (via TPDO)

Configuration: (ToDo)


For the contollers, you should extend the controller AND provide with proper configuration and urdf.
The controller can be a child class of canopen_ros2_controllers::CanopenProxyController. Following function should be filled:
- command_interface_configuration
- state_interface_configuration

Configuration: (ToDo)

How does the controller connect with the hardware interface? How are the states and the commands transferred?

We have registered the command and state interfaces in the hardware interface. By claiming the same resources in the controller, the controller is able to read data and write command. We should be aware of the indices we use.

https://github.com/ros-industrial/ros2_canopen/blob/master/canopen_ros2_controllers/src/canopen_proxy_controller.cpp


https://github.com/StoglRobotics-forks/ros2_canopen/blob/master-galactic-backport/canopen_ros2_controllers/src/cia402_device_controller.cpp
 
