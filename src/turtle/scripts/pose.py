#!/usr/bin/env python3
# _*_ coding: utf-8 _*

import rospy
from turtlesim.msg import Pose

def main():
    rospy.init_node('pose_node')
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback, queue_size=10)
    rospy.spin()
    
def callback(msg):
    rospy.loginfo("X: %f, Y: %f, Theta: %f, Linear Velocity: %f, Angular Velocity: %f", msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity)
if __name__ == '__main__':
    main()
