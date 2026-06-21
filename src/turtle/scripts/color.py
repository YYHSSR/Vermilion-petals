#!/usr/bin/env python3
# _*_ coding: utf-8 _*

import rospy
import sys

def main():
    rospy.init_node('color_node')
    if len(sys.argv) != 4:
        rospy.logerr("请提供3个RGB参数 (0-255)")
        sys.exit(1)
    
    try:
        rospy.set_param('/turtlesim/background_r', int(sys.argv[1]))
        rospy.set_param('/turtlesim/background_g', int(sys.argv[2]))
        rospy.set_param('/turtlesim/background_b', int(sys.argv[3]))
    except ValueError:
        rospy.logerr("参数必须是整数")
        sys.exit(1)
        
if __name__ == '__main__':
    main()  