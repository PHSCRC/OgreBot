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

POLL_TIME=0.01
def callback(cmd_vel):
    global my_drive, vels
#    print("linearx", cmd_vel.linear.x)
#    print("angularz", cmd_vel.angular.z)
    linear = (cmd_vel.linear.x * ENCODER_COUNTS_PER_RADIAN)/ WHEEL_RADIUS
    angular = cmd_vel.angular.z * (ROBOT_RADIUS/WHEEL_RADIUS) * ENCODER_COUNTS_PER_RADIAN
    leftMotorSpeed = linear + angular #Get linear speed of each wheel in m/s
    rightMotorSpeed = linear - angular#Get linear speed of each wheel
 #   print(leftMotorSpeed)
  #  print(rightMotorSpeed)
    my_drive.axis0.controller.vel_setpoint = -leftMotorSpeed
    my_drive.axis1.controller.vel_setpoint = rightMotorSpeed

def turn(rad):
    global my_drive
    my_drive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    my_drive.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    speed = my_drive.axis0.controller.config.vel_limit
    my_drive.axis0.controller.config.vel_limit = 5000
    my_drive.axis1.controller.config.vel_limit = 5000
    countsToMove=rad.data*(ROBOT_RADIUS/WHEEL_RADIUS) * ENCODER_COUNTS_PER_RADIAN
    print("rad", rad.data)
    print("counts", countsToMove)
    my_drive.axis0.controller.pos_setpoint = my_drive.axis0.encoder.pos_estimate + countsToMove
    my_drive.axis1.controller.pos_setpoint = my_drive.axis1.encoder.pos_estimate +  countsToMove
    time.sleep(10)
   # while (my_drive.axis0.encoder.vel_estimate < 100):
   #     pass
    my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    my_drive.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    my_drive.axis0.controller.config.vel_limit = speed
    my_drive.axis1.controller.config.vel_limit = speed

def drive(distance):
    global my_drive
    my_drive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    my_drive.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    speed = my_drive.axis0.controller.config.vel_limit
    my_drive.axis0.controller.config.vel_limit = 5000
    my_drive.axis1.controller.config.vel_limit = 5000
    countsToMove=(distance.data * ENCODER_COUNTS_PER_RADIAN)/WHEEL_RADIUS
    print("distance", distance.data)
    print("counts", countsToMove)
    my_drive.axis0.controller.pos_setpoint = my_drive.axis0.encoder.pos_estimate - countsToMove
    my_drive.axis1.controller.pos_setpoint = my_drive.axis1.encoder.pos_estimate +  countsToMove
    time.sleep(10)
   # while (my_drive.axis0.encoder.vel_estimate < 100):
   #     pass
    my_drive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    my_drive.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    my_drive.axis0.controller.config.vel_limit = speed
    my_drive.axis1.controller.config.vel_limit = speed

def listener():
    global my_drive, vels
   # print("looking for odrive")
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

    rospy.Subscriber("/turn", Float64, turn)
    rospy.Subscriber("/drive", Float64, drive)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    my_drive = None
    vels = None
    listener()
