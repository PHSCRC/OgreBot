#!/usr/bin/env python

import time
import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import *

# distances[0] is actually 90 degrees
def wallfollower():
    print("wallfollower")
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
            rospy.sleep(0.2)
            turnRightDegrees(90)
            rospy.sleep(1)
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
        moveForwardDistance(0.1)
        rospy.sleep(0.5)
        # alignToWall()



'''
For the beginning, it might be better look to at the back half of the robot to alignToWall, but this
will only work for the beginning.
    -Stephane
'''
def alignToWall():
    global distances
    global angle_increment
    minDist = distances[90]
    minDistAngle = 90 
# this is bad rn
    print(distances)
    for i in range(-45, 45):
        if distances[i] < minDist:
            minDist = distances[i]
            minDistAngle = i
    print("Moving to " + str(minDistAngle) + "degrees")
    if minDistAngle < 0:
        turnLeftDegrees(-math.fabs(minDistAngle))
    else:
        turnRightDegrees(-minDistAngle)
    rospy.sleep(1)

def setup():
    global distances
    global angle_increment
    global vel
    global turn, drive
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler)
    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    turn = rospy.Publisher('turn', Float64, queue_size=10)
    drive = rospy.Publisher('drive', Float64, queue_size=10)
    time.sleep(1)
#    turnLeftDegrees(31)
#    time.sleep(2)
#    turnRightDegrees(180)
#    time.sleep(2) 
#    moveForwardDistance(.5)
#    time.sleep(2)
#    moveForwardDistance(-.5)
#    time.sleep(2)
#    time.sleep(1)
    alignToWall()
    wallfollower()
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
        if (not math.isinf(distances[dist])) :
            d.append(distances[dist])

        else :
            d.append(1000)

    distances = d
    angle_increment = scan.angle_increment

    #print(distances)

# Moves forward m meters at .20
def moveForwardDistance(distance):
    global drive
    drive.publish(Float64(distance))

# Turns left at 1 radians (? degrees) per second
def turnLeftDegrees(degrees):
    radians = - degrees * (math.pi / (180))
    global turn
    turn.publish(Float64(radians))

# Turns right at 1 radians (? degrees) per second
def turnRightDegrees(degrees):
    global turn
    radians = degrees * (math.pi/180)
    turn.publish(Float64(radians))
    #
   
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
    turn = None
    drive = None
    angle_increment = None
    vel = None
    distances = []
    setup()
