I would like to validate the positional accuracy of each chopstick tip relative to itself

Home position is when the chopstick is normal to each linear plate

The script that implements the inverse kinematics that I've derived is "dual_chopstick_validation"

There are multiple things that have been tested and validated:

    Chopstick movement via the pitch and yaw dynamixels on the linear plate

    Linear movement via the dynamixels in extended position mode under the large plate driving the leadscrews

    Reading serial input from a microswitch in order to "home" the linear axes and reset the zero position upon initialization 

Unfortunately, all of these have been tested independently and haven't been restructured into classes and definitions or unified into a single script and made compatible with ROS 

    This will need to happen at a later date, especially with the addition of the Force/Torque sensors


To get the optitrack_validation.py script running, here is what needs to happen:

    Plug in power and USB to the U2D2 board and flip the switch

    Ensure that the four dynamixels plugged in have power 

    The two pitch dynamixel IDs are 2 and 5
    the two yaw dynamixel IDs are 3 and 6

Note: there isn't any code for the linear axis dynamixels, and they aren't even plugged in cause I haven't been able to validate all of them together. Additionally, one of them crapped out last week and I haven't replaced it yet...



