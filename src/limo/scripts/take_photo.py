#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def image_callback(msg):
    bridge = CvBridge()
    try:
        # 将 ROS 图像转换为 OpenCV 格式
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # 保存图片
        filename = "robot_photo.jpg"
        cv2.imwrite(filename, cv_img)
        rospy.loginfo("成功拍照并保存为: " + filename)
        # 拍照成功后关闭节点
        rospy.signal_shutdown("Photo saved successfully.")
    except CvBridgeError as e:
        print("CvBridge Error:", e)

if __name__ == '__main__':
    rospy.init_node('take_photo_node', anonymous=True)
    # 订阅相机 RGB 图像话题
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.loginfo("正在等待相机图像流并拍照...")
    rospy.spin()

