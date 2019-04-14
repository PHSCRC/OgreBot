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

'''
def read_flame () :
    global ard
    read = ard.readline().decode().strip()
    return read

'''
def toggle_extinguisher(state):
    if(state):
        GPIO.output(21, GPIO.HIGH)
    else:
        GPIO.output(21, GPIO.LOW)

def removeInf(distances) :
    d = []
    for dist in range(len(distances)) :
        if (not math.isinf(distances[dist])) :
            d.append(distances[dist])

        else :
            d.append(1000)
    return d

def infToAdj () :
    # For each infinity, searches backwards and forwards for nearest found value and sets it to that value
    global distances
    distances = removeInf(distances)

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


def alignToWall(n, coneSize = 40) :

    dist = infToAdj()
#    print("ALIGNING TO WALL")
#    print(dist)
    minDistSum = 10000 # Arbitrary high value to find mins
    minDistAngle = 0

    numValuesTaken = 7
    #debugList = []
    for i in range(n - coneSize, n + coneSize - numValuesTaken) :
        #debugList.append(dist[i])
        sum = 0
        for j in range(0, numValuesTaken) :
            sum += dist[i + j]

        if sum < minDistSum :
            minDistSum = sum
            minDistAngle = i + numValuesTaken // 2

    print("Moving to " + str(minDistAngle) + "degrees")

    wheelOffset = 0
    minDistAngle += wheelOffset
    if minDistAngle < 0:
        turnLeftDegrees(n-math.fabs(minDistAngle))
    else:
        turnRightDegrees(n-minDistAngle)
    #print(debugList)
    rospy.sleep(0.5)

    

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
    '''
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
    '''
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




# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan):
    global distances, detectOpening, angle_increment, justTurned, soundStart, tcs, inRoom, acceptTime
    if rospy.get_time()-scan.header.stamp.secs<acceptTime and soundStart:
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
        r,g,b,c = tcs.get_raw_data()
        # For the room detection
        if (inRoom):
            print('got white')
        print('inRoom', inRoom)
        if(inRoom==1):#r+g+b>300
            turnAndMove(0,0)
            moveForward(0.2)
            rospy.sleep(1)
           #fireSweep()
            print("fire sweep")
            slowturnRightDegrees(540)
#            turnRightDegrees(360)
            rospy.sleep(1)
            print("after turn")
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
            justTurned=True
            #turnRightDegrees(180)
            #rospy.sleep(0.5)
            #moveForwardDistance(0)
        elif(justTurned):
            alignToWall(0)
            rospy.sleep(0.7)
            inRoom=0
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
