#!/usr/bin/env python
import time
import serial
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
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

def setup():
    global distances, angle_increment, turn, vel, drive
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler, queue_size=1, buff_size=1)
    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    turn = rospy.Publisher('turn', Float64, queue_size=10)
    drive = rospy.Publisher('drive', Float64, queue_size=10)
    run()
    time.sleep(1)
    rospy.spin()


def run() :
    global distances

    forwardSpeed = 0.05
    turns = 0

    while True :

        if (distances[90] < 0.3 and distances[90] != 1000) :

            turns += 1

            if (turns == 6) :
                turnLeftDegrees(180)
                turns = 0

            elif (turns == 3) :
                turnRightDegrees(180)

            elif (turns > 3) :
                turnLeftDegrees(90)

            else :
                turnRightDegrees(90)

            rospy.sleep(1)


        else :
            moveForward(forwardSpeed)


def scanHandler(scan):
    global distances, detectOpening, angle_increment, justTurned, acceptTime
    if rospy.get_time()-scan.header.stamp.secs < acceptTime and soundStart:
        distances = scan.ranges
        print("scanhandler")
        d = []
        newDist = 0
        for dist in range(len(distances)) :
            #print(distances[dist])
            if (not math.isinf(distances[dist])) :
                d.append(distances[dist])

            else :
                d.append(1000)

        distances = d
        angle_increment = scan.angle_increment

        if len(detectOpening)==0 or justTurned:
            detectOpening = [distances[0], distances[0], distances[0]]

        # Distance from right wall, 1000 = inf right now
        if (distances[0] != 1000):
            newDist = distances[0]


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
