import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from ogrebot.msg import robot_vels
import tf

from collections import deque
import math
from math import sin, cos, pi

SMOOTHING_ARRAY=[-0.19048, -0.04762, 0.09524, 0.23810, 0.38095, 0.52381]

def deadReckonCallback(motor_vels):
    if polledTime is None:
        polledTime=rospy.Time.now()
    else:
        polledTime+=motorVels.POLL_TIME

    lastLeftPoints.rotate(-1)
    lastRightPoints.rotate(-1)

    lastLeftPoints[len(SMOOTHING_ARRAY)-1] = motor_vels.left_vel
    lastRightPoints[len(SMOOTHING_ARRAY)-1] = motor_vels.right_vel

    smoothedLeftVel=0
    smoothedRightVel=0
    for i in range(len(SMOOTHING_ARRAY)):
        smoothedLeft+=SMOOTHING_ARRAY[i]*lastLeftPoints[i]
        smoothedRight+=SMOOTHING_ARRAY[i]*lastRightPoints[i]

    smoothedLeft*=2*pi #to rad/s
    smoothedRight*=2*pi #to rad/s

    v=(Vr+Vl)/2
    vtheta=(smoothedRightVel-smoothedLeftVel)/(2*motor_vels.ROBOT_RADIUS)
    firstPart=motor_vels.POLL_TIME*v*(2+cos(motor_vels.POLL_TIME*vtheta/2))/3
    x+=motor_vels.POLL_TIME*v*firstPart*cos(theta+motor_vels.POLL_TIME*vtheta/2)
    y+=motor_vels.POLL_TIME*v*firstPart*sin(theta+motor_vels.POLL_TIME*vtheta/2)

    odom_quat=tf.transformations.quaternion_from_euler(0,0,theta)
    odom_broadcaster.sendTransform(
        (x,y,0.0),
        odom_quat,
        polledTime,
        "base_link",
        "odom"
    )

    odom=Odometry()

    odom.header.stamp=polledTime
    odom.header.frame_id="odom"

    odom.pose.pose=Pose(Point(x,y,0.0),Quaternion(*odom_quat))

    odom.child_frame_id="base_link"
    odom.twist.twist=Twist(Vector3(vx,vy,0.0), Vector3(0.0,0.0,vtheta))

    odom_pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node("odometry_publisher")

    odom_pub=rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster= tf.TransformBroadcaster()

    rospy.Subscriber("/wheel_vels", robot_vels, callback)

    x=0.0
    y=0.0
    th=0.0

    vx=0.0
    vy=0.0
    vth=0.0

    lastLeftPoints=deque([0.0]*len(SMOOTHING_ARRAY))
    lastRightPoints=deque([0.0]*len(SMOOTHING_ARRAY))

    polledTime=None

    rospy.spin()
