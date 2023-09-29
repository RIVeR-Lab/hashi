#!/usr/bin/env python3

import numpy as np
import os
import rospy
import sys
from std_msgs.msg import Int32MultiArray
from threading import Lock

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
    ADDR_GOAL_VELOCITY          = 104
    LEN_GOAL_VELOCITY           = 4

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
DXL4_ID                     = 4                 # Dynamixel#1 ID : 4
DXL5_ID                     = 5                 # Dynamixel#1 ID : 5
DXL6_ID                     = 6                 # Dynamixel#1 ID : 6

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupVelSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
# Create lock to avoid conflict with read write position locks
read_write_lock = Lock()

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

DXL_IDS = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID, DXL5_ID, DXL6_ID]
# Enable torques for each of the Dynamixels
for dxl_id in DXL_IDS:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % dxl_id)
    

DEGREE_TO_POSITION_VALUE = (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE)/360.0

# Synchronously set the positions of all motors in DXL_IDS to the corresponding position in motor_positions
# List[Int] -> void
def sync_write_stick_positions(msg: Int32MultiArray):
    raw_positions = msg.data
    motor_positions = raw_positions[:4]
    # l-r right, u-d right, l-r left, u-d left
    motor_indices = [1,2,4,5] # Note: These indices are -1 since lists are 0-indexed
    for i in range(len(motor_positions)):
        goal_position = motor_positions[i]
        dxl_id = DXL_IDS[motor_indices[i]]
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)), DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]
        dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
            quit()
    # Add the linear rail junk
    r, l = sync_write_carriage_positions(raw_positions[4], raw_positions[5])
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(r)), DXL_HIBYTE(DXL_LOWORD(r)), DXL_LOBYTE(DXL_HIWORD(r)), DXL_HIBYTE(DXL_HIWORD(r))]
    dxl_addparam_result = groupSyncWrite.addParam(1, param_goal_position)        
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(l)), DXL_HIBYTE(DXL_LOWORD(l)), DXL_LOBYTE(DXL_HIWORD(l)), DXL_HIBYTE(DXL_HIWORD(l))]
    dxl_addparam_result = groupSyncWrite.addParam(4, param_goal_position)
    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    # Now do the rai update
    # sync_write_carriage_positions(raw_positions[4], raw_positions[5])

def write_single_pos(dxl_id: int, goal_position: float) -> None:
    # with read_write_lock:
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)), DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]
    dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
        quit()
    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

def zero_motor_positions():
    with read_write_lock:
        for i in range(len(DXL_IDS)):
            dxl_id = DXL_IDS[i]
            goal_position = 2048
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position)), DXL_LOBYTE(DXL_HIWORD(goal_position)), DXL_HIBYTE(DXL_HIWORD(goal_position))]
            dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
                quit()
        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

def sync_write_carriage_positions(r_inc, l_inc):
    # Split the incoming data message
    # r_inc, l_inc = msg.data
    # Get the current servo position
    r_rail_pos = get_servo_pos(0)
    # Write the updated position value
    # write_single_pos(1, r_rail_pos + r_inc)
    # Now do the same thing for the left servo
    l_rail_pos = get_servo_pos(3)
    # # Write the updated position value
    # write_single_pos(4, l_rail_pos + l_inc)
    return r_rail_pos + r_inc,  l_rail_pos + l_inc

def write_single_vel(dxl_id: int, goal_velocity: float) -> None:
    # TODO - This doesn't work at all
    with read_write_lock:
        print(f'ID: {dxl_id}, VEL: {goal_velocity}')
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, 320)
        print(dxl_comm_result)
        if dxl_comm_result != COMM_SUCCESS:
            print("Error communicating with the servo")
        elif dxl_error != 0:
            print("Servo error:", packetHandler.getTxRxResult(dxl_comm_result))
        else:
            print("Goal velocity set successfully")
    
def get_servo_pos(i):
    DXL_ID =  DXL_IDS[i]
    # with read_write_lock:
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    return dxl_present_position

if __name__ == "__main__":
    rospy.init_node('MoveMotor', anonymous = True)
    stick_sub = rospy.Subscriber("/teleop/stick_commands", Int32MultiArray, callback=sync_write_stick_positions, queue_size=1)
    # carriage_sub = rospy.Subscriber("/teleop/carriage_commands", Int32MultiArray, callback=sync_write_carriage_positions, queue_size=10)
    zero_motor_positions()
    rospy.spin()
    
    

    
