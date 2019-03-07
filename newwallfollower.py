#!/usr/bin/env python

import time
import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def newwallfollower():
    global distances
    global turning
    Running = True
    tolerance = 0.7


    while Running:
        # Distance from right wall
        newDist = distances[0]

        # This determines if there is an opening to the right
        if (newDist > tolerance):
            turning = True
            print("Detected opening")
            # Distance will have to be determined through testing
            moveForwardDistance(0.05)
            print("Moving into opening")
            # Distance will have to be determined through testing
            moveForwardDistance(0.05)
            turning = False

def turnRight90():
    # lincoln make this work
    pass

def setup():
    global vels
    rospy.init_node('wallfollower', anonymous=False)
    vels = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    newwallfollower()
    rospy.Subscriber("/scan", LaserScan, scanHandler)
    rospy.spin()

# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan):
    global angleadjust
    global distances
    global angle_increment
    global turning
    distances = scan.ranges
    inf = 1000
    d = []
    for dist in range(len(distances)) :
        #print(distances[dist])
        if (type(distances[dist]) is int) :
            d.append(distances)
        else :
            d.append(1000)

    print(len(distances))
    print(distances[0])
    print(distances[180])
    if not (turning) :
        if (distances[0]>distances[180]):
            angleadjust = .3
        elif (distances[180]>distances[0]):
            angleadjust = -.3

    drive(.2, angleadjust)


#SPEED IS IN M/S
def drive(linspeed, angspeed):
    global vels
    msg = Twist()
    msg.linear = Vector3(linspeed, 0, 0)
    msg.angular = Vector3(0, 0, angspeed)
    vels.publish(msg)

def moveForward(speed):
    global vels
    msg = Twist()
    msg.linear = Vector3(speed, 0, 0)
    msg.angular = Vector3(0, 0, 0)
    vels.publish(msg)

def moveBackward(speed):
    global vels
    msg = Twist()
    msg.linear = Vector3(-speed, 0, 0)
    msg.angular = Vector3(0, 0, 0)
    vels.publish(msg)

def turnLeft(speed):
    global vels
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, -speed)
    vels.publish(msg)

def turnRight(speed):
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, speed)
    vel.publish(msg)


if __name__ == '__main__':
    angleadjust = 0;
    angle_increment = None
    vels = None
    distances = None
    turning = False
    setup()
