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
toggle_extinguisher=None
tolerance = .30
tSize = 5
distances = []
detectOpening = [];
forwardSpeed = 0.1 #m/s
pGain = -0.000
acceptTime = 1
latestFireReading = None

justTurned = False
soundStart = False
tcs = Adafruit_TCS34725.TCS34725()

inRoom = False

def fireSweep(fireReading):
    global inRoom, soundStart
    if (fireReading > 300):
        soundStart = True
    else:
        if (inRoom):
            latestFireReading = fireReading

def extinguisher():
    print("in extinguisher")

def alignToWall(n) :
    global distances, angle_increment, detectOpening
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
    rospy.sleep(0.25)



def colorHandler(data) :
    global inRoom
    if (data.data) :
        inRoom = 1
    else:
        inRoom = 0
    print("in color handler: "+str(inRoom))

def setup() :
    global distances, angle_increment, turn, vel, drive, fireReading, toggle_extinguisher
    rospy.init_node('wallfollower', anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanHandler, queue_size=1, buff_size=1)
    rospy.Subscriber("/color", Bool, colorHandler)
    vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    turn = rospy.Publisher('turn', Float64, queue_size=10)
    drive = rospy.Publisher('drive', Float64, queue_size=10)
    rospy.Subscriber('/arduinoInfo', Integer, fireSweep)
    toggle_extinguisher=ospy.Publisher('/toggle_extinguisher', Empty, queue_size=10)
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
    newDist = 0
    for dist in range(len(distances)) :
        #print(distances[dist])
        if (not math.isinf(distances[dist])) :
            d.append(distances[dist])

        else :
            d.append(1000)
    return d


# gives list of distances starting from angle 0 to 360, at increment of angle_increment
def scanHandler(scan) :
    global distances, detectOpening, angle_increment, justTurned, soundStart, tcs, inRoom, acceptTime, latestFireReading, toggle_extinguisher
    if rospy.get_time()-scan.header.stamp.secs<acceptTime and soundStart:
        distances = scan.ranges
        print("scanhandler")

        angle_increment = scan.angle_increment

        distances = removeInf(distances)

        if not len(detectOpening) or justTurned:
            detectOpening = [distances[0], distances[0], distances[0]]

        # Distance from right wall, 1000 = inf right now
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
            #fireSweep()

            print("Fire sweep")
#            turnRightDegrees(360)
            turnLeftDegrees(180)
            rospy.sleep(1)

            print("After turn")
            for i2 in range(0, 12):
                turnRightDegrees(30)
                rospy.sleep(1)
                if (latestFireReading) :
                    oldReading=latestFireReading
                    turnRightDegrees(30)
                    rospy.sleep(1)
                    if (oldReading < latestReading):
                        while (oldReading < latestReading) :
                            turnRightDegrees(30)
                            rospy.sleep(1)
                        while (oldReading < latestReading) :
                            turnLeftDegrees(5)
                            rospy.sleep(1)
                        turnRightDegrees(5)
                        toggle_extinguisher.publish()
                        rospy.sleep(10)
                        toggle_extinguisher.publish()
                    else:
                        oldReading, latestReading = latestReading, oldReading
                        while(oldReading<latestReading)
                            turnLeftDegrees(30)
                            rospy.sleep(1)
                        while(oldReading<latestReading):
                            turnRightDegrees(5)
                            rospy.sleep(1)
                        turnLeftDegrees(5)
                        toggle_extinguisher.publish()
                        rospy.sleep(10)
                        toggle_extinguisher.publish()
                    break



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
            if(distances[0]<distances[180]):
                alignToWall(0)
            else:
                alignToWall(180)
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
