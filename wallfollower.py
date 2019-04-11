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
newDist = None
justTurned = False
soundStart = False
tcs = Adafruit_TCS34725.TCS34725()

inRoom = False

ard = None

def read_flame () :
    global ard

    read = ard.readline().decode().strip()
    return read

def toggle_extinguisher(state):
    if(state):
        GPIO.output(21, GPIO.HIGH)
    else:
        GPIO.output(21, GPIO.LOW)

def alignToWall(n):
    global distances, angle_increment, detectOpening
    minDist = distances[n]
    minDistAngle = n
    # this is bad rn
    #print(distances)
    debugAlign = []
    for i in range(n-25, n+25):
        debugAlign.append(distances[i])
        if distances[i] < minDist:
            minDist = distances[i]
            minDistAngle = i
    print("Moving to " + str(minDistAngle) + "degrees")
    print(debugAlign)
    if minDistAngle < 0:
        turnLeftDegrees(n-math.fabs(minDistAngle))
    else:
        turnRightDegrees(n-minDistAngle)
    rospy.sleep(0.25)

def alignToClosestWall() :
    global distances, angle_increment, detectOpening
    minIndex = distances.index(min(distances))
    print("Moving to " + str(minIndex) + "degrees")
    turnLeftDegrees(minIndex)
    rospy.sleep(0.25)

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
    global distances, angle_increment, turn, vel, drive, fireReading, ard, turnslow
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
    turnslow = rospy.Publisher('turnslow', Float64, queue_size=10)
    drive = rospy.Publisher('drive', Float64, queue_size=10)
    time.sleep(1)
    rospy.spin()

def removeInf(distances) :
    d = []
    for dist in range(len(distances)) :
        if (not math.isinf(distances[dist])) :
            d.append(distances[dist])

        else :
            d.append(1000)
    return d


# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan) :
    global distances, detectOpening, angle_increment, justTurned, tcs, inRoom, acceptTime
    if rospy.get_time()-scan.header.stamp.secs<acceptTime:
        distances = scan.ranges
        print("scanhandler")
        print("Flame sensor reading: " + str(read_flame()))

        angle_increment = scan.angle_increment

        distances = removeInf(distances)

        if not len(detectOpening) or justTurned:
            detectOpening = [distances[0], distances[0], distances[0]]

        # Distance from right wall, 1000 = inf right now
        newDist = 0
        if (distances[0] != 1000):
            newDist = distances[0]

        r,g,b,c = tcs.get_raw_data()

        # For the room detection
        print('inRoom', inRoom)
        if (inRoom) :#r+g+b>300


            print ("Got White")
            turnAndMove(0, 0)
            moveForward(0.2)
            rospy.sleep(1)


            print("Fire sweep")
            oldRead = -1
            latestRead = -1
            slowturnRightDegrees(540)
            maxRead = -1

            fireInRoom = False


            numQuestionableReads = 0

            initialTime = rospy.get_time()
            while (rospy.get_time() - initialTime < 20) : # for 10 seconds
                latestRead = read_flame()
                if not latestRead :
                    continue
                else :
                    latestRead = int(latestRead)
                print("Latest: " + str(latestRead))
                print("Max: " + str(maxRead))
                if latestRead > maxRead:
                    maxRead = latestRead
                if (latestRead > 200) :
                    fireInRoom = True

                if (fireInRoom) :
                    if (maxRead - latestRead > 5) :
                        numQuestionableReads += 1
                        if numQuestionableReads > 20 :

                            turnAndMove(0, 0)
                            toggle_extinguisher(True)
                            print("Extinguish on")
                            rospy.sleep(10)
                            toggle_extinguisher(False)
                            print("Extinguish off")
                            break

                oldRead = latestRead

            ##### Align to closest wall might mess with the rest of the code.
            #alignToWall(0)
            #alignToClosestWall()
            rospy.sleep(1)
            i = 0
            while (inRoom==1) :
                print("getting out of room")
                print(i * 0.05)
                moveForwardDistance(0.05)
                rospy.sleep(0.7)
                i += 1
            moveForwardDistance(0.05)
            rospy.sleep(0.7)
            print("out of room?")
            print(inRoom)
            turnAndMove(0,0)
#            alignToWall(0)
#            justTurned=True
            #turnRightDegrees(180)
            #rospy.sleep(0.5)
            #moveForwardDistance(0)
        elif(justTurned):
            print("In justTurned")
            if(distances[0]<distances[180]):
                alignToWall(0)
            else:
                alignToWall(180)
            rospy.sleep(0.7)
            justTurned=False
        else:
            # This determines if there is an opening to the right
            #if (newDist > (sum(detectOpening)/len(detectOpening)) + tolerance) :

            print("Not in room")
            if distances[90]<0.32:
                print("Turning left")
                turnLeftDegrees(90)
                rospy.sleep(1)
            elif (newDist > .65 or newDist > (sum(detectOpening)/len(detectOpening)) + tolerance):
                print("Detected opening")
                # Distance will have to be determined through testing
                turnAndMove(0,0)
                if(newDist>(sum(detectOpening)/len(detectOpening))+tolerance):
                    moveForwardDistance(0.15)
                    rospy.sleep(1)
                turnRightDegrees(90)
                rospy.sleep(1)
                print("Moving into opening")
                # Distance will have to be determined through testing
                moveForwardDistance(0.4)
                rospy.sleep(1)
                justTurned=True
                turnAndMove(0,0)
            else:
              #  if(abs(distances[]-distances[180])>0.1):
                   # alignToWa
                #if(18<24-distances[0]<30):
                #    alignToWall(0)
                p=24-distances[0]
                turnAndMove(forwardSpeed, p*pGain)
                detectOpening.append(newDist)

            # To keep it from overflowing memory
            if (len(detectOpening) > tSize) :
                detectOpening.pop(0)


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

# Turns left at 1 radians (? degrees) per second
def slowturnLeftDegrees(degrees):
    global turnslow
    radians = - degrees * (math.pi / (180))
    turnslow.publish(Float64(radians))

# Turns right at 1 radians (? degrees) per second
def slowturnRightDegrees(degrees):
    global turnslow
    radians = degrees * (math.pi/180)
    turnslow.publish(Float64(radians))
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
