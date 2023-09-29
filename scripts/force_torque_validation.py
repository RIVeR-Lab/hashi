#!/usr/bin/env python3

"""Script to move the stewart platform to various configurations 
to OptiTrack system measurements to desired poses"""

import json
import tqdm
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R
from hashi.srv import HashiCommand, HashiCommandResponse

import tf
import tf_conversions
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

collected_msgs = []
open_pose = [2048, 2047, 2042, 2048, 0, 0]
close_pose = [2048, 1821, 2042, 2048, 0, 0]
def ft_callback(msg: WrenchStamped):
    collected_msgs.append({
        'time': msg.header.stamp.to_nsec(),
        'f_x': msg.wrench.force.x,
        'f_y': msg.wrench.force.y,
        'f_y': msg.wrench.force.z,
        't_x': msg.wrench.torque.x,
        't_y': msg.wrench.torque.y,
        't_z': msg.wrench.torque.z
    })

def dump_to_json(shore_hardness: str):
    print(f'Saving {len(collected_msgs)} messages')
    with open(f'/home/river/{shore_hardness}.json', 'w') as json_file:
        json.dump(collected_msgs, json_file)


if __name__ == '__main__':
    rospy.init_node('FTValidator', anonymous = True)
    shore_hardness = rospy.get_param('~shore')
    pub_raw = rospy.Publisher('/hashi/commands/raw', Int32MultiArray)
    sub_ft = rospy.Subscriber('/ft', WrenchStamped, ft_callback, queue_size=1)

    for _ in range(10):
        # Close chopstick
        pub_raw.publish(Int32MultiArray(data=close_pose))
        rospy.sleep(0.5)
        # Open chopstick
        pub_raw.publish(Int32MultiArray(data=open_pose))
        rospy.sleep(0.5)
    dump_to_json(shore_hardness)