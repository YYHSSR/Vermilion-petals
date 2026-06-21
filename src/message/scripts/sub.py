#! /usr/bin/env python3
# _*_ coding: utf-8 _*

import rospy
from std_msgs.msg import String
from message.msg import content

def callback(msg):
    rospy.loginfo("I heard: 姓名:%s 年龄:%d 身高:%f", msg.name, msg.age, msg.height)

def sub():
    rospy.init_node("sub_node")
    sub = rospy.Subscriber("chatter",content,callback,queue_size=10)

if __name__ == "__main__":
    sub()
    rospy.spin()
