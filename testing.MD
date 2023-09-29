# Steps for Kinematic Validation

### Plug in device
- USB to QTPy
- USB to U2D2 Board
- Power to U2D2 Board

### Start the VRPN Service
`roslaunch vrpn_client_ros sample.launch`

### Run the main service mode
`rosrun hashi dual_chopstick_validation_ros.py`

### Calculate the initial platform position
`rosrun hashi zero_platform_and_record_origin_tf.py`

### Broadcast the zero position
`rosrun hashi broadcast_platform_zero_transform.py`

### Run one of the test scripts
