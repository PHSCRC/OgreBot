#!/usr/bin/env python
import odrive
from odrive.enums import *

import time
import math

import rospy
from geometry_msgs.msg import Twist
from ogrebot.msg import robot_vels

ROBOT_RADIUS = .1 #in meteres
WHEEL_RADIUS = .08 # in meters
ENCODER_COUNTS_PER_ROTATION = 12*4096

POLL_TIME=0.01
def callback(cmd_vel):
    global my_drive, vels
    print("linearx", cmd_vel.linear.x)
    print("angularz", cmd_vel.angular.z)
    leftWheelSpeed = cmd_vel.linear.x - (ROBOT_RADIUS*cmd_vel.angular.z) #Get linear speed of each wheel in m/s
    rightWheelSpeed = cmd_vel.linear.x + (ROBOT_RADIUS*cmd_vel.angular.z)#Get linear speed of each wheel
    leftMotorSpeed = (2*math.pi*WHEEL_RADIUS*leftWheelSpeed)#Get rotational speed of each wheel in rotations per second
    rightMotorSpeed = (2*math.pi*WHEEL_RADIUS*rightWheelSpeed)#Get rotational speed of each wheel
    print(leftMotorSpeed)
    print(rightMotorSpeed)
    my_drive.axis0.controller.vel_setpoint = -leftMotorSpeed*ENCODER_COUNTS_PER_ROTATION
    my_drive.axis1.controller.vel_setpoint = rightMotorSpeed*ENCODER_COUNTS_PER_ROTATION


def poll(event):
    global my_drive, vels
    leftReading = my_drive.axis0.encoder.vel_estimate/ENCODER_COUNTS_PER_ROTATION
    rightReading = my_drive.axis1.encoder.vel_estimate/ENCODER_COUNTS_PER_ROTATION
    msg = robot_vels()
    msg.left_vel = leftReading
    msg.right_vel = rightReading
    msg.POLL_TIME = .01
    msg.ROBOT_RADIUS = ROBOT_RADIUS
    vels.publish(msg)

def listener():
    global my_drive, vels
    print("looking for odrive")
    my_drive = odrive.find_any()
    my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    my_drive.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while my_drive.axis0.current_state != AXIS_STATE_IDLE:
        rospy.sleep(0.1)
    my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    my_drive.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    vels = rospy.Publisher('wheel_vels', robot_vels, queue_size=10)
    rospy.Timer(rospy.Duration(POLL_TIME), poll, oneshot=False)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    my_drive = None
    vels = None
    listener()
