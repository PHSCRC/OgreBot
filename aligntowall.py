#!/usr/bin/env python

import time
import math
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import *
import Adafruit_TCS34725
import smbus

turn = None
drive = None
angle_increment = None
vel = None
tolerance = .30
tSize = 5
distances = []
detectOpening=[];
forwardSpeed=0.1 #m/s
pGain=-0.000

justTurned=False
soundStart=True
tcs=Adafruit_TCS34725.TCS34725()

inRoom=False


'''
For the beginning, it might be better look to at the back half of the robot to alignToWall, but this
will only work for the beginning.
    -Stephane
'''
def alignToWall(n):
    global distances, angle_increment, detectOpening
    minDist = distances[n]
    minDistAngle = n
    for i in range(n-45, n+45):
        if distances[i] < minDist:
            minDist = distances[i]
            minDistAngle = i
    print("Moving to " + str(minDistAngle) + "degrees")
    if minDistAngle < 0:
        turnLeftDegrees(-math.fabs(minDistAngle))
    else:
        turnRightDegrees(-minDistAngle)
    rospy.sleep(0.25)

def colorHandler(data):
    global inRoom
    inRoom = data.data
    print("in color handler: "+str(inRoom))

def setup():
    global distances, angle_increment, turn, vel, drive
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler, queue_size=1, buff_size=1)
    rospy.Subscriber("/color", Bool, colorHandler)
    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    turn = rospy.Publisher('turn', Float64, queue_size=10)
    drive = rospy.Publisher('drive', Float64, queue_size=10)
    time.sleep(1)
    rospy.spin()

def scanHandler(scan):
    global distances, detectOpening, angle_increment, justTurned, soundStart, tcs, inRoom
    distances = scan.ranges
    d = []
    for dist in range(len(distances)):
        if (not math.isinf(distances[dist])):
            d.append(distances[dist])
        else:
            d.append(1000)
    distances = d
    angle_increment = scan.angle_increment
    if rospy.get_time()-scan.header.stamp.secs<1 and soundStart:
        alignToWall(0)
    rospy.sleep(.05)
# Moves forward m meters at .20
def moveForwardDistance(distance):
    global drive
    drive.publish(Float64(distance))

# Turns left at 1 radians (? degrees) per second
def turnLeftDegrees(degrees):
    global turn
    radians = - degrees * (math.pi / (180))
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

def turnAndMove(speedForward, speedClockwise):
    global vel
    msg = Twist()
    msg.linear = Vector3(speedForward, 0, 0)
    msg.angular = Vector3(0, 0, speedClockwise)
    vel.publish(msg)


def stopeverything():
    print('exiting')
    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()
    msg.linear = Vector3(0, 0, 0)
    msg.angular = Vector3(0, 0, 0)
    vel.publish(msg)
    print('done exiting')

if __name__ == '__main__':
    print(rospy.get_published_topics())
    while not (['/motor_listener_listening', 'std_msgs/String'] in rospy.get_published_topics()):
        pass
    print('starting')
    rospy.on_shutdown(stopeverything)
    setup()
