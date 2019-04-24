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
import RPi.GPIO as GPIO

turn = None
drive = None
angle_increment = None
vel = None

tolerance = .30
tSize = 5
updatedDistances = []
detectOpening = [];
forwardSpeed = 0.1 #m/s
pGain = -0.000
newDist = 0
justTurned = False
soundStart = False
tcs = Adafruit_TCS34725.TCS34725()

inRoom = False

ard = None

def alignToWall(n) :
    global distances, angle_increment, detectOpening
    minDist = distances[n]
    minDistAngle = n
    for i in range(n-25, n+25):
        if distances[i] < minDist:
            minDist = distances[i]
            minDistAngle = i
    print("Moving to " + str(minDistAngle) + "degrees")
    if minDistAngle < 0:
        turnLeftDegrees(n-math.fabs(minDistAngle))
    else:
        turnRightDegrees(n-minDistAngle)
    rospy.sleep(0.25)


def infToAdj () :
    # For each infinity, searches backwards and forwards for nearest found value and sets it to that value
    global updatedDistances
    distances = removeInf(updatedDistances)

    adj = []
    for i in range(0, len(distances)) :
        if distances[i] != 1000 : #not math.isinf(distances[i]) :
            adj.append(distances[i])

        else :
            j = 0
            while distances[(i + j) % 360] == 1000 and distances[i - j] == 1000 :  #(math.isinf(distances[(i + j) % 360])) and (math.isinf(distances[i - j])) :
                j += 1
            adj.append(min(distances[(i + j) % 360], distances[i - j]))

    return adj

def setup() :
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(21, GPIO.OUT)
    global distances, angle_increment, turn, vel, drive, fireReading, ard
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
        if 'USB2.0-Serial' in desc:
            print('arduino connected')
            ard = serial.Serial(port, 9600, timeout=0)
            while True:
                curr = ard.readline().decode().strip()
#                print(curr)
                if curr == 'sound':
                    print('got sound')
                    break
    time.sleep(1)
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler, queue_size=1, buff_size=1)
    rospy.Subscriber("/color", Bool, colorHandler)
    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    turn = rospy.Publisher('turn', Float64, queue_size=10)
    drive = rospy.Publisher('drive', Float64, queue_size=10)
    time.sleep(1)
    alignToWallExp(0, 40)
    moveAround()
    rospy.spin()

def removeInf(distances) :
    d = []
    for dist in range(len(distances)) :
        if (not math.isinf(distances[dist])) :
            d.append(distances[dist])

        else :
            d.append(1000)
    return d

def scanHandler(scan):
    global updatedDistances, detectOpening, newDist
    if rospy.get_time()-scan.header.stamp.secs< 1:
        distances = scan.ranges

        angle_increment = scan.angle_increment

        updatedDistances = removeInf(distances)
        if not len(detectOpening) or justTurned:
            detectOpening = [updatedDistances[0], updatedDistances[0], updatedDistances[0]]

        # Distance from right wall, 1000 = inf right now
        newDist = 0
        if (distances[0] != 1000):
            newDist = updatedDistances[0]

def moveAround() :
    global updatedDistances, detectOpening, justTurned, inRoom, tolerance, newDist
    while True:
        distances = updatedDistances
        if(justTurned):
            alignToWallExp(0, 40)
            rospy.sleep(0.7)
            justTurned=False
        else:
            if distances[90]<0.32:
                print("Turning left")
                turnLeftDegrees(90)
                rospy.sleep(1)
            elif (newDist > .65 or newDist > (sum(detectOpening)/len(detectOpening)) + tolerance):
                print("Detected opening")
                # Distance will have to be determined through testing
                turnAndMove(0,0)
                if(newDist>(sum(detectOpening)/len(detectOpening))+tolerance):
                    moveForwardDistance(0.1)
                    rospy.sleep(1)
                turnRightDegrees(90)
                rospy.sleep(1)
                print("Moving into opening")
                # Distance will have to be determined through testing
                moveForwardDistance(0.3)
                rospy.sleep(1)
                justTurned=True
                turnAndMove(0,0)
            else:
                p=24-distances[0]
                turnAndMove(forwardSpeed, p*pGain)
                detectOpening.append(newDist)

            # To keep it from overflowing memory
            if (len(detectOpening) > tSize) :
                detectOpening.pop(0)
    rospy.sleep(0.5)

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
