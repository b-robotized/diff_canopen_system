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
 
