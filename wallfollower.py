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
pGain=0
justTurned=False
soundStart=True
tcs=Adafruit_TCS34725.TCS34725()

recievedWhite=False
justWentIntoRoom=False
timeWentIntoRoom=0


'''
For the beginning, it might be better look to at the back half of the robot to alignToWall, but this
will only work for the beginning.
    -Stephane
'''
def alignToWall(n):
    global distances, angle_increment, detectOpening
    minDist = distances[90]
    minDistAngle = 90
# this is bad rn
    print(distances)
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
    global recievedWhite
    recievedWhite = data.data
    print(recievedWhite)

def setup():
    global distances, angle_increment, turn, vel, drive
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler, queue_size=1, buff_size=1)
    rospy.Subscriber("/color", Bool, colorHandler)
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
#    turnAndMove(0, 0)
#    alignToWall(0)
#    time.sleep(1)
    #wallfollower()
    rospy.spin()

# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan):
    global distances, detectOpening, angle_increment, justTurned, soundStart, tcs, recievedWhite, justWentIntoRoom, timeWentIntoRoom
    if justWentIntoRoom and rospy.get_time()-timeWentIntoRoom>1.5:
        justWentIntoRoom=False
    if rospy.get_time()-scan.header.stamp.secs<1 and soundStart:
        distances = scan.ranges

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

        if len(detectOpening)==0:
            detectOpening = [distances[0], distances[0], distances[0]]

        # Distance from right wall, 1000 = inf right now
        if (distances[0] != 1000):
            newDist = distances[0]
        r,g,b,c = tcs.get_raw_data()
        # For the room detection
        if (recievedWhite):
            print('got white')
        print('recievedWhite', recievedWhite)
        if(recievedWhite):#r+g+b>300
           #fireSweep()
            print("fire sweep")
            moveForward(0.15)
            rospy.sleep(0.2)
#            turnRightDegrees(360)
            rospy.sleep(0.2)
            turnLeftDegrees(180)
            rospy.sleep(0.4)
            moveForward(0.38)
            rospy.sleep(0.2)
            justWentIntoRoom=True
            timeWentIntoRoom=rospy.get_time()
        else:
            # This determines if there is an opening to the right
            #if (newDist > (sum(detectOpening)/len(detectOpening)) + tolerance) :
            if (newDist > .65 or newDist > (sum(detectOpening)/len(detectOpening)) + tolerance):
                print("Detected opening")
                # Distance will have to be determined through testing
                moveForwardDistance(0.1)
                rospy.sleep(1)
                turnRightDegrees(90)
                rospy.sleep(1)
                print("Moving into opening")
                # Distance will have to be determined through testing
                moveForwardDistance(0.3)
                rospy.sleep(1)
                justTurned=True
            elif distances[90]<0.21:
                turnLeftDegrees(90)
                rospy.sleep(0.4)
            else:
              #  if(abs(distances[]-distances[180])>0.1):
                   # alignToWall(0)
                p=pGain*(distances[0]-distances[180])
                turnAndMove(forwardSpeed, p)
                detectOpening.append(newDist)

            # To keep it from overflowing memory
            if (len(detectOpening) > tSize) :
                detectOpening.pop(0)

        # This aligns regularly
        print("distances[0]: " + str(distances[0]) + ", distances[90]: " + str(distances[90]) + ", distances[180]: " + str(distances[180]) + ", distances[270]: " + str(distances[270]))

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
