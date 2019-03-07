#!/usr/bin/env python

import time
import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

# distances[0] is actually 90 degrees
def wallfollower():
    global distances
    Running = True;

    sinceLastAlign = 0
    # alignToWall()

    tolerance = 10
    tSize = 5
    detectOpening = [distances[0], distances[0], distances[0]]

    while Running:
        # Distance from right wall
        newDist = distances[0]

        # This determines if there is an opening to the right
        if (newDist > sorted(detectOpening)[int(len(detectOpening) / 2)] + tolerance) :
            print("Detected opening")
            # Distance will have to be determined through testing
            moveForwardDistance(0.05)
            turnRightDegrees(90)
            print("Moving into opening")
            # Distance will have to be determined through testing
            moveForwardDistance(0.05)
            detectOpening = [distances[0]]
        else :
            detectOpening.append(newDist)

        # To keep it from overflowing memory
        if (len(detectOpening) > tSize) :
            detectOpening.pop(0)

        # This aligns regularly
        moveForwardDistance(1)
        # alignToWall()



'''
For the beginning, it might be better look to at the back half of the robot to alignToWall, but this
will only work for the beginning.
    -Stephane
'''
def alignToWall():
    global distances
    global angle_increment
    minDist = distances[0]
    minDistAngle = 0
    for i in range(int(len(distances) / 4)):
        if distances[i] < minDist:
            minDist = distances[i]
            minDistAngle = angle_increment * i
    if minDistAngle < 90:
        turnLeftDegrees(90 - minDistAngle)
    else:
        turnRightDegrees(minDistAngle - 90)

def alighnToWall

def setup():
    global distances
    global angle_increment
    global vel
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler)
    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    time.sleep(1)
    print('forward')
   # moveForwardDistance(1)
    time.sleep(1)
    print('back')
   # moveForwardDistance(-1)
    time.sleep(1)
    print('left')
    turnLeftDegrees(90)
    time.sleep(1)
    print('right')
    turnRightDegrees(90)
    #wallfollower()
    rospy.spin()

# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan):
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

    distances = d
    angle_increment = scan.angle_increment

    #print(distances)

# Moves forward m meters at .20
def moveForwardDistance(distance):
    time = distance / 0.2
    moveForward(0.2)
    #
    rospy.sleep(time)
    moveForward(0)

# Turns left at 1 radians (? degrees) per second
def turnLeftDegrees(degrees):
    radians = degrees * (math.pi / (180))
    turnLeft(1)
    print('1')
    #
    rospy.sleep(radians)
    print('2')
    turnLeft(0)
    print('3')

# Turns right at 1 radians (? degrees) per second
def turnRightDegrees(degrees):
    radians = degrees * (math.pi / 180)
    turnRight(1)
    #
    rospy.sleep(radians)
    turnRight(0)

#SPEED IS IN M/S
def moveForward(speed):
    global vel
    msg = Twist()
    msg.linear = Vector3(speed, 0, 0)
    msg.angular = Vector3(0, 0, 0)
    vel.publish(msg)

def moveBackward(speed):
    global vel
    msg = Twist()
    msg.linear = Vector3(-speed, 0, 0)
    msg.angular = Vector3(0, 0, 0)
    vel.publish(msg)

def turnLeft(speed):
    global vel
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, -speed)
    vel.publish(msg)

def turnRight(speed):
    global vel
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, speed)
    vel.publish(msg)


if __name__ == '__main__':
    print('change is working')
    angle_increment = None
    vel = None
    distances = []
    setup()
