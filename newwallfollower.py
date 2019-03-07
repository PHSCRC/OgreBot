#!/usr/bin/env python

import time
import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def setup():
    rospy.init_node('wallfollower', anonymous=False)
    vels = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    time.sleep(1)
    drive(.2, 0)
    time.sleep(1)
    drive(-.2, 0)
    time.sleep(1)
    drive(.2, 1)
    time.sleep(1)
    drive(.2, -1)    
    rospy.Subscriber("/scan", LaserScan, scanHandler)
    rospy.spin()

# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan):
    global angleadjust
    global distances
    global angle_increment
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
    print(distnace[180])
    if (distances[0]>distances[180]):
        angleadjust += 1
    elif (distances[180]>distances[0]):
        angleadjust -= 1

    drive(.2, angleadjust)


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
    angleadjust = 0;
    angle_increment = None
    vels = None
    distances = None
    setup()
