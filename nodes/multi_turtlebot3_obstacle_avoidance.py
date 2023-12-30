#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

dist = 0.7
def callback0(msg):
    if msg.ranges[0] > dist and msg.ranges[30] > dist and msg.ranges[330] > dist: #checking at 0, 30, 330 degrees
        vel0.linear.x = 0.4
        vel0.angular.z = 0
    else :
        vel0.linear.x = 0
        vel0.angular.z = 0.5
        if msg.ranges[0] > dist and msg.ranges[30] > dist and msg.ranges[330] > dist:
            vel0.linear.x = 0.4
            vel0.angular.z = 0 
    pub0.publish(vel0)

def callback1(msg):
    if msg.ranges[0] > dist and msg.ranges[30] > dist and msg.ranges[330] > dist: #checking at 0, 30, 330 degrees
        vel1.linear.x = 0.4
        vel1.angular.z = 0
    else :
        vel1.linear.x = 0
        vel1.angular.z = 0.5
        if msg.ranges[0] > dist and msg.ranges[30] > dist and msg.ranges[330] > dist:
            vel1.linear.x = 0.4
            vel1.angular.z = 0 
    pub1.publish(vel1)

def callback2(msg):
    if msg.ranges[0] > dist and msg.ranges[30] > dist and msg.ranges[330] > dist: #checking at 0, 30, 330 degrees
        vel2.linear.x = 0.4
        vel2.angular.z = 0
    else :
        vel2.linear.x = 0
        vel2.angular.z = 0.5
        if msg.ranges[0] > dist and msg.ranges[30] > dist and msg.ranges[330] > dist:
            vel2.linear.x = 0.4
            vel2.angular.z = 0 
    pub2.publish(vel2)

rospy.init_node("obs",anonymous=True)
pub0 = rospy.Publisher("tb3_0/cmd_vel", Twist, queue_size=10)
pub1 = rospy.Publisher("tb3_1/cmd_vel", Twist, queue_size=10)
pub2 = rospy.Publisher("tb3_2/cmd_vel", Twist, queue_size=10)
vel0 = Twist()
vel1 = Twist()
vel2 = Twist()
sub0 = rospy.Subscriber("tb3_0/scan", LaserScan, callback0)
sub1 = rospy.Subscriber("tb3_1/scan", LaserScan, callback1)
sub2 = rospy.Subscriber("tb3_2/scan", LaserScan, callback2)
rospy.spin()