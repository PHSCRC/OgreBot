#!/usr/bin/env python

import time
import math

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

def alignToWall():
    global distances
    global angle_increment
    minDist = distances[0]
    minDistAngle = 0
    for i in range(int(len(distances) / 4)):
        if distances[i] < minDist:
            minDist = distances[i]
            minDistAngle = 0.01745329251 * i
    if minDistAngle < 90:
        turnLeftDegrees(90 - minDistAngle)
    else:
        turnRightDegrees(minDistAngle - 90)

def newwallfollower():
    global distances
    global turning
    Running = True
    tolerance = 0.7
    # newDist starts as [0, 0, 0]
    newDist = [0, 0, 0]
    while Running:
        # Distance from right wall
        if (not math.isinf(distances[0])):
            newDist.append(distances[0])
            newDist.remove(newDist[0])
        print('newdist', newDist)
        # This determines if there is an opening to the right
        if ((sum(newDist)/3) > tolerance):
            turning = True
            print("Detected opening. Turning equals True.")
            # Distance will have to be determined through testing
            moveForwardDistance(0.05)
            print("Moving into opening")
            # Distance will have to be determined through testing
            moveForwardDistance(0.05)
            turning = False
	    print("Turning equals false.")

def turnRight90():
    # lincoln make this work
    pass

def newAlignToWall():
    newDist = [distances[0],distances[0],distances[0]]
    avgDist = sum(newDist)/3
    minDist = sum(newDist)/3
    turnRight(0.5)
    # tolerance = 0.005
    tolerance = 0
    while (True):
        if (not math.isinf(distances[0])):
            newDist.append(distances[0])
            newDist.remove(newDist[0])
            avgDist = sum(newDist)/3
        print(minDist, ", ", distances[0],", ", avgDist)
        minDist = avgDist
        if (avgDist > minDist + tolerance):
            break
    turnRight(0)
    avgDist = [distances[0],distances[0],distances[0]]
    turnLeft(0.5)
    while (distances[0] <= minDist):
        if (not math.isinf(distances[0])):
            newDist.append(distances[0])
            newDist.remove(newDist[0])
            avgDist = sum(newDist)/3
        print(minDist, ", ", distances[0],", ", avgDist)
        minDist = avgDist
        if (avgDist > minDist + tolerance):
            break
    turnLeft(0)
    print("Aligned to wall")

def setup():
    global vels
    rospy.init_node('wallfollower', anonymous=True)
    vels = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, scanHandler)
    time.sleep(1)
    # turnRight(0)
    alignToWall()
    # newwallfollower()
    rospy.spin()

# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan):
    global angleadjust
    global distances
    global angle_increment
    global turning
    distances = scan.ranges
    inf = -1
    d = []
    for dist in range(len(distances)) :
        #print(distances[dist])
        if (type(distances[dist]) is float) :
            d.append(distances[dist])
        else :
            #print("KSDFJLKSDFLKJSDKFLJSKLF")
            #print(type(distances[dist]))
            #print(distances[dist])
            d.append(0)#distances[dist - 1])

    distances = d
    #print(distances)
    #print('distance 0', distances[0])
   # print(distances[180])
    if not (turning) :
        if (distances[0]>distances[180]):
            angleadjust = .4
        elif (distances[180]>distances[0]):
            angleadjust = -.4

   # drive(.2, angleadjust)


#SPEED IS IN M/S
def drive(linspeed, angspeed):
    global vels
    msg = Twist()
    msg.linear = Vector3(linspeed, 0, 0)
    msg.angular = Vector3(0, 0, angspeed)
    vels.publish(msg)

# Moves forward m meters at .20
def moveForwardDistance(distance):
    time = distance / 0.2
    moveForward(0.2)
    #
    rospy.sleep(time)
    moveForward(0)

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
    global vels
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, speed)
    vels.publish(msg)


if __name__ == '__main__':
    angleadjust = 0;
    angle_increment = None
    vels = None
    distances = None
    turning = False
    print("hi")

    setup()
