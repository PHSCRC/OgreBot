#!/usr/bin/env python
import odrive
from odrive.enums import *

import time
import math

import rospy
from geometry_msgs.msg import Twist
from ogrebot.msg import robot_vels
from std_msgs.msg import *

ROBOT_RADIUS = .097 #in meteres
WHEEL_RADIUS = .045 # in meters
ENCODER_COUNTS_PER_RADIAN = 1303.7972938088 #(2048 * 4)/(2*math.pi)

def poll(event):
    global my_drive, vels
    leftReading = my_drive.axis0.encoder.vel_estimate/ENCODER_COUNTS_PER_RADIAN
    rightReading = my_drive.axis1.encoder.vel_estimate/ENCODER_COUNTS_PER_RADIAN
    msg = robot_vels()
    msg.left_vel = leftReading
    msg.right_vel = rightReading
    msg.POLL_TIME = .01
    msg.ROBOT_RADIUS = ROBOT_RADIUS
    vels.publish(msg)
    
def setup():
    global my_drive, vels
    my_drive = odrive.find_any()
    print('found odrive')
    rospy.init_node('vel_listener', anonymous=True)
    vels = rospy.Publisher('wheel_vels', robot_vels, queue_size=10)
    rospy.Timer(rospy.Duration(POLL_TIME), poll, oneshot=False)
    rospy.spin()

if __name__ == '__main__':
    my_drive = None
    vels = None
    setup()
