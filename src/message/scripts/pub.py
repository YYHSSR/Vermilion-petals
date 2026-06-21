#! /usr/bin/env python3
# _*_ coding: utf-8 _*

import rospy
from std_msgs.msg import String
from message.msg import content

def pub():
    rospy.init_node("pub_node")
    pub = rospy.Publisher("chatter",content,queue_size=10)
    msg = content()  
    msg.name = "zf"
    msg.age = 21
    msg.height = 160 
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Publisher interrupted")

if __name__ == "__main__":
    pub()

