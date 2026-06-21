#!/usr/bin/env python3
# _*_ coding: utf-8 _*

import rospy
from geometry_msgs.msg import Twist
import sys

global vel
vel = Twist()

class Motion:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=3)
        self.flag = True

    def run_time(self, lv, av, tim):
        vel = Twist()
        vel.linear.x = lv
        vel.angular.z = av
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < tim and not rospy.is_shutdown():
            self.vel_pub.publish(vel)
            rate.sleep()
        self.vel_pub.publish(Twist())

    def run_time_ang(self, lv, av_start, av_end, tim):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < tim and not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(elapsed / tim, 1.0)
            av = av_start + (av_end - av_start) * progress
            vel = Twist()
            vel.linear.x = lv
            vel.angular.z = av
            self.vel_pub.publish(vel)
            rate.sleep()
        self.vel_pub.publish(Twist())

    def callback(self, event):      
        self.flag = False

    def move_cl(self):
        rospy.Timer(rospy.Duration(3), self.callback, oneshot=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.flag:
                rospy.loginfo("开始直线运行")
                vel.linear.x = 0.4
                vel.angular.z = 0
            else:
                rospy.loginfo("开始圆运行")
                vel.linear.x = 0.4
                vel.angular.z = -0.3  
            self.vel_pub.publish(vel)
            rate.sleep() 

    def move_circle(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("开始圆运行")
            vel.linear.x = 0.4
            vel.angular.z = -0.3  
            self.vel_pub.publish(vel)
            rate.sleep() 

    def move_line(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.loginfo("开始直线运行")
            vel.linear.x = 0.4
            vel.angular.z = 0
            self.vel_pub.publish(vel)
            rate.sleep() 
    
    def move_ellipse(self):
        while not rospy.is_shutdown():
            rospy.loginfo("开始椭圆运行")
            self.run_time_ang(1, -0.2,-0.6, 4)
            self.run_time_ang(1, -0.6,-0.2, 4)

    def move_8(self):
        while not rospy.is_shutdown():
            rospy.loginfo("开始8字运行")
            self.run_time(0.4, 0.5, 12.5)
            self.run_time(0.4, -0.5, 12.5)    

    def move_square(self):
        while not rospy.is_shutdown():
            rospy.loginfo("开始正方形运行")
            self.run_time(0.6,0,3)
            self.run_time(0, -0.4,3.9)

    def choose_motion(self):
        if len(sys.argv) != 2:
            rospy.logerr("请输入正确的运动模式")
            sys.exit(1)
        elif sys.argv[1] == "cl":
            self.move_cl()
        elif sys.argv[1] == "circle":
            self.move_circle()
        elif sys.argv[1] == "line":
            self.move_line()
        elif sys.argv[1] == "ellipse":
            self.move_ellipse()
        elif sys.argv[1] == "8":
            self.move_8()
        elif sys.argv[1] == "square":
            self.move_square()
        else:
            rospy.logerr("请输入正确的运动模式")
            sys.exit(1)

if __name__ == '__main__':
    try:
        rospy.init_node('motion_node')
        motion = Motion()
        motion.choose_motion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


