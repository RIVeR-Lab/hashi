#!/usr/bin/env python3
import rospy
import socket
import sys
import math
import numpy as np
from hashi.srv import HashiCommand
from hashi.msg import Teleop
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy

class JoyStick:
    """
    Connect and control the HASHI end effector with a joystick
    """
    def __init__(self, joy_addr = '/dev/input/js0'):
        self.joy_addr = joy_addr
        self.base_motor_command_sub = rospy.Subscriber("/joy", Joy, self.process_joy)
        self.stick_pub = rospy.Publisher("/teleop/stick_commands", Int32MultiArray, queue_size=10)
        self.carriage_pub = rospy.Publisher("/teleop/carriage_commands", Int32MultiArray, queue_size=10)
        self.hashi_command = rospy.ServiceProxy('hashi_control', HashiCommand)
        self.Z_START_LEFT = 230
        self.Z_START_RIGHT = 230
        self.Z_INC = 5
        self.maxspd = 500
        self._servo_min = 1821
        self._servo_max = 2275
        self._rail_command = 4096 * 2 # Two rotations
        self.l_pub = rospy.Publisher('/hashi/commands/left', Teleop)
        self.r_pub = rospy.Publisher('/hashi/commands/right', Teleop)
        self.pub_raw = rospy.Publisher('/hashi/commands/raw', Int32MultiArray)
        # This variable restrict if we are using the tool position or controlling a raw axis
        self.use_toolpoint = True
        self.hold_left = False
        self.hold_right = False

    def process_joy(self, msg):
        # print(msg)
        # LEFT JOYSTICK
        l_up_down = msg.axes[1]
        l_right_left = msg.axes[0] * -1
        # LEFT CARRIAGE LINEAR
        l_carriage_up = msg.buttons[4]
        l_carriage_down = msg.axes[2]
        # RIGHT JOYSTICK
        r_up_down = msg.axes[4] * -1
        r_right_left = msg.axes[3] 
        # RIGHT CARRIAGE LINEAR
        r_carriage_up = msg.buttons[5]
        r_carriage_down = msg.axes[5]
        # Only change if the button does not match our previous value
        if bool(msg.buttons[0]):
            self.use_toolpoint = not self.use_toolpoint
        if bool(msg.buttons[1]):
            print('holding left!')
            self.hold_left = not self.hold_left
        if bool(msg.buttons[2]):
            print('holding right!')
            self.hold_right = not self.hold_right

        # If we are using the toolpoint
        if self.use_toolpoint:
            self.process_sticks(l_up_down, l_right_left, r_up_down, r_right_left, l_carriage_up, l_carriage_down, r_carriage_up, r_carriage_down)
        else:
            # we are tracking the xyz raw
            self.process_sticks_raw(l_up_down, l_right_left, r_up_down, r_right_left, l_carriage_up, l_carriage_down, r_carriage_up, r_carriage_down)

    def process_sticks_raw(self, l_up_down: float, l_right_left: float, r_up_down: float, r_right_left: float, l_carriage_up, l_carriage_down, r_carriage_up, r_carriage_down):
        '''
        Parse joysticks into positional commands for the chopsticks
        '''
        # l-r right, u-d right, l-r left, u-d left
        # These match the servo orderings on the physical robot
        commands = [r_right_left, r_up_down, l_right_left, l_up_down]
        # Normalize joystick values into motors commands
        servo_pos = [self.remap_position_raw(command) for command in commands]
        # Generate the rail commands
        l, r = self.process_carriage_raw(l_carriage_up, l_carriage_down, r_carriage_up, r_carriage_down)
        servo_pos.append(l)
        servo_pos.append(r)
        self.pub_raw.publish(Int32MultiArray(data=servo_pos))


    def process_carriage_raw(self, l_carriage_up: float, l_carriage_down: float, r_carriage_up: float, r_carriage_down: float):
        '''
        Change the position of the chopsticks linear axes
        '''
        ### LEFT CARRIAGE
        if l_carriage_up == 1 and l_carriage_down < 1.0:
            # Skip the linear axes
            l_command = 0
        elif l_carriage_up == 1:
            l_command = -self._rail_command
        elif l_carriage_down < 1:
            l_command = self._rail_command
        else:
            l_command = 0
        ### RIGHT CARRIAGE
        if r_carriage_up == 1 and r_carriage_down < 1.0:
            # Skip the linear axes
            r_command = 0
        elif r_carriage_up == 1:
            r_command = -self._rail_command
        elif r_carriage_down < 1:
            r_command = self._rail_command
        else:
            r_command = 0
        return r_command, l_command


    def remap_position(self, value: float, min_pos: float, max_pos: float) -> float:
        # Normalize stick value from -1 to 1
        processed_val =  int(((value - (-1)) / (1 - (-1))) * (max_pos - min_pos) + min_pos)
        # print(f'Original Value: {value}, Processed Value: {processed_val}')
        return processed_val
    
    def remap_position_raw(self, value: float) -> float:
        # Normalize stick value from -1 to 1
        processed_val =  int(((value - (-1)) / (1 - (-1))) * (self._servo_max - self._servo_min) + self._servo_min)
        # print(f'Original Value: {value}, Processed Value: {processed_val}')
        return processed_val
        
    def process_sticks(self, l_up_down: float, l_right_left: float, r_up_down: float, r_right_left: float, l_carriage_up, l_carriage_down, r_carriage_up, r_carriage_down):
        '''
        Parse joysticks into positional commands for the chopsticks
        '''
        # l-r right, u-d right, l-r left, u-d left
        # These match the servo orderings on the physical robot
        commands = [r_right_left, r_up_down, l_right_left, l_up_down]
        # Normalize joystick values into motors commands
        left_y = self.remap_position(l_right_left, -50, 50)
        left_x = -self.remap_position(l_up_down, -50, 50)
        right_y = self.remap_position(r_right_left, -50, 50)
        right_x = -self.remap_position(r_up_down, -50, 50)
        # Generate the rail commands
        right_z, left_z = self.process_carriage(l_carriage_up, l_carriage_down, r_carriage_up, r_carriage_down)
        # Send left stick postion
        r_c = Teleop()
        r_c.x, r_c.y, r_c.z, r_c.stick = left_x, left_y, left_z, 1
        l_c = Teleop()
        l_c.x, l_c.y, l_c.z, l_c.stick = right_x, right_y, right_z, 0
        # Publish both messages
        if not self.hold_right:
            if self.use_toolpoint:
                self.r_pub.publish(r_c)
            else:
                self.r_pub_raw.publish(r_c)
        else:
            print('skipping right')
        if not self.hold_left:
            if self.use_toolpoint:
                self.l_pub.publish(l_c)
            else:
                self.l_pub_raw.publish(l_c)
        else:
            print('skipping left')
        # res = self.hashi_command(left_x, left_y, left_z, 0, 0, 0, 1)
        # Send right stick position
        # res = self.hashi_command(right_x, right_y, right_z, 0, 0, 0, 0)


    def process_carriage(self, l_carriage_up: float, l_carriage_down: float, r_carriage_up: float, r_carriage_down: float):
        '''
        Change the position of the chopsticks linear axes
        '''
        ### LEFT CARRIAGE
        if l_carriage_up == 1 and l_carriage_down < 1.0:
            # Skip the linear axes
            l_command = 0
        elif l_carriage_up == 1:
            l_command = 1
        elif l_carriage_down < 1:
            l_command = -1
        else:
            l_command = 0
        ### RIGHT CARRIAGE
        if r_carriage_up == 1 and r_carriage_down < 1.0:
            # Skip the linear axes
            r_command = 0
        elif r_carriage_up == 1:
            r_command = 1
        elif r_carriage_down < 1:
            r_command = -1
        else:
            r_command = 0
        
        self.Z_START_RIGHT += r_command * self.Z_INC
        self.Z_START_LEFT += l_command * self.Z_INC
        self.Z_START_RIGHT = np.clip(self.Z_START_RIGHT, 225, 260)
        self.Z_START_LEFT = np.clip(self.Z_START_LEFT, 225, 260)
        return self.Z_START_RIGHT, self.Z_START_LEFT


if __name__ == "__main__":
    rospy.init_node("joystick_node")
    node = JoyStick()
    rospy.spin()

