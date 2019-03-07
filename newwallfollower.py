#!/usr/bin/env python

import time
import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def setup():
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler)
    vels = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.spin()

# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan):
    distances = scan.ranges
    angle_increment = scan.angle_increment
    

#SPEED IS IN M/S
def drive(linspeed, angspeed):
    msg = Twist()
    msg.linear = Vector3(linspeed, 0, 0)
    msg.angular = Vector3(0, 0, angspeed)
    vel.publish(msg)

def moveForward(speed):
    msg = Twist()
    msg.linear = Vector3(speed, 0, 0)
    msg.angular = Vector3(0, 0, 0)
    vel.publish(msg)

def moveBackward(speed):
    msg = Twist()
    msg.linear = Vector3(-speed, 0, 0)
    msg.angular = Vector3(0, 0, 0)
    vel.publish(msg)

def turnLeft(speed):
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, -speed)
    vel.publish(msg)

def turnRight(speed):
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, speed)
    vel.publish(msg)


if __name__ == '__main__':
    angle_increment = None
    vels = None
    distances = None
    setup()
