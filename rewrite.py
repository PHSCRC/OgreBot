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
distances = []
detectOpening = [];
forwardSpeed = 0.1 #m/s
pGain = -0.000
acceptTime = 1

justTurned = False
soundStart = False
tcs = Adafruit_TCS34725.TCS34725()

inRoom = False

ard = None

def read_flame () :
    global ard
    return ard.readline().decode().strip()

def toggle_extinguisher(state):
    if(state):
        GPIO.output(21, GPIO.HIGH)
    else:
        GPIO.output(21, GPIO.LOW)

def alignToWall(n) :
    global distances, angle_increment, detectOpening
    print("Align to Wall")
    minDist = distances[n]
    minDistAngle = n
    # this is bad rn
    #print(distances)
    for i in range(n-25, n+25):
        if distances[i] < minDist:
            minDist = distances[i]
            minDistAngle = i
    print("Moving to " + str(minDistAngle) + "degrees")
    if minDistAngle < 0:
        turnLeftDegrees(n-math.fabs(minDistAngle))
    else:
        turnRightDegrees(n-minDistAngle)
    rospy.sleep(1)



def colorHandler(data) :
    global inRoom
    if (data.data) :
        inRoom = 1
    else:
        inRoom = 0
    print("in color handler: "+str(inRoom))

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

def removeInf(distances) :
    d = []
    for dist in range(len(distances)) :
        #print(distances[dist])
        if (not math.isinf(distances[dist])) :
            d.append(distances[dist])

        else :
            d.append(-1)
    return d


def sweepRoom () :

    oldRead = read_flame()
    latestRead = 0

    for i2 in range(0, 12):
        turnRightDegrees(30)
        rospy.sleep(1)
        latestRead = read_flame()

        if (latestRead > 100) :

            if (oldRead < latestRead):
                while (oldRead < latestRead) :
                    turnRightDegrees(30)
                    rospy.sleep(1)
                    oldRead = latestRead
                    latestRead = read_flame()

                oldRead, latestRead = latestRead, oldRead
                while (oldRead < latestRead) :
                    turnLeftDegrees(5)
                    rospy.sleep(1)
                    oldRead = latestRead
                    latestRead = read_flame()

                turnRightDegrees(5) # ?
                toggle_extinguisher(True)
                rospy.sleep(10)
                toggle_extinguisher(False)


            else:
                oldRead, latestRead = latestRead, oldRead
                while (oldRead < latestRead) :
                    turnLeftDegrees(30)
                    rospy.sleep(1)
                    oldRead = latestRead
                    latestRead = read_flame()


                oldRead, latestRead = latestRead, oldRead
                while (oldRead < latestRead) :
                    turnRightDegrees(5)
                    rospy.sleep(1)
                    oldRead = latestRead
                    latestRead = read_flame()

                turnLeftDegrees(5) # ?
                toggle_extinguisher(True)
                rospy.sleep(10)
                toggle_extinguisher(False)

            break

        oldRead = latestRead

def planeDistance (angle) :
    global distances
    hypo = distances[angle]
    if hypo != -1 :
        x = hypo * math.cos(angle * angle_increment) # convert to radians
        y = hypo * math.sin(angle * angle_increment)

    else :
        x = -1
        y = -1

    return [x, y]



def scanHandler(scan) :
    global distances, detectOpening, angle_increment, justTurned, tcs, inRoom, acceptTime

    if rospy.get_time()-scan.header.stamp.secs<acceptTime:
        distances = scan.ranges
        print("scanhandler")

        angle_increment = scan.angle_increment
        r,g,b,c = tcs.get_raw_data()

        distances = removeInf(distances)

        # For the room detection
        print('inRoom', inRoom)
        if (inRoom) :#r+g+b>300

            print ("Now in room")
            turnAndMove(0, 0)
            moveForward(0.2)
            rospy.sleep(1)

            print("Turning back to the door")
            turnLeftDegrees(180)
            rospy.sleep(1)

            print("Sweeping")
            sweepRoom()

            alignToWall(0)


            while (inRoom) :
                print("getting out of room")
                moveForwardDistance(0.05)
                rospy.sleep(0.7)
            moveForwardDistance(0.05)
            rospy.sleep(0.7)

            print("Got out of room")
            print("inRoom should be false, and is " + str(inRoom))
            turnAndMove(0, 0)
            justTurned = True
            #turnRightDegrees(180)
            #rospy.sleep(0.5)
            #moveForwardDistance(0)
        elif (justTurned):
            turnAndMove(0, 0)
            if(distances[0] < distances[180]):
                alignToWall(0)
            else:
                alignToWall(180)
            rospy.sleep(0.7)
            justTurned = False

        else:
            # This determines if there is an opening to the right
            #if (newDist > (sum(detectOpening)/len(detectOpening)) + tolerance) :

            print("0: " + str(distances[0]))
            print("90: " + str(distances[90]))
            print("180: " + str(distances[180]))
            if (distances[0] > 0.65 or distances[0] == -1) :
                print("Turn right")
                turnAndMove(0,0)

                xy315 = planeDistance(315)
                while (xy315[1] > -0.1) :
                    moveForwardDistance(0.01) # Arbitrary, to make sure robot clears opening
                    rospy.sleep(0.5)
                    xy315 = planeDistance(315)

                rospy.sleep(1)

                turnRightDegrees(90)
                rospy.sleep(1)
                print("Moving into opening")
                # Distance will have to be determined through testing
                moveForwardDistance(0.3)
                rospy.sleep(1)
                justTurned = True
                turnAndMove(0,0)
               # alignToWall(0)

            elif distances[90] > 0.4 or distances[90] == -1:
                print("Go Forward")

                moveForward(0.05)
                rospy.sleep(1)

            else:
                print("Turn left")

                turnAndMove(0, 0)
                moveForwardDistance(0.1) # Arbitrary, to make sure robot clears opening
                rospy.sleep(1)
                turnLeftDegrees(90)
                rospy.sleep(1)
                print("Moving into opening")
                # Distance will have to be determined through testing
                moveForwardDistance(0.3)
                rospy.sleep(1)
                justTurned = True
                turnAndMove(0,0)
                #alignToWall(0)

        # This aligns regularly
        #print("distances[0]: " + str(distances[0]) + ", distances[90]: " + str(distances[90]) + ", distances[180]: " + str(distances[180]) + ", distances[270]: " + str(distances[270]))

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
