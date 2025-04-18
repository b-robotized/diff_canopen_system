# rpm_powertrain_driver

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
 

## Testing with fake  diff drive system

### Setup the vcan0 interface
```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
```


### Start teh `diff_drive_system` with fake motor controllers
```
ros2 launch diff_canopen_system fake_diff_drive_system.launch.xml
```


# TODOs to make driver more robust
- don't crash where there is no can device
- don't crash if data can not be parsed


