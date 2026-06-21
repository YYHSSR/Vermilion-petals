#! /usr/bin/env python3
# _*_ coding: utf-8 _*

import rospy
import sys
from message.srv import text,textRequest,textResponse

def client():
    rospy.init_node("client_node")
    client = rospy.ServiceProxy("text",text)
    if len(sys.argv) != 4:
        rospy.logerr("请正确提交参数")
        sys.exit(1)
    # 方式1:
    # rospy.wait_for_service("text")
    # 方式2
    client.wait_for_service()
    req = textRequest()
    req.num1 = int(sys.argv[1])
    req.num2 = int(sys.argv[2]) 
    req.age = float(sys.argv[3])
    resp = client.call(req)

if __name__ == "__main__":
    client()
    rospy.loginfo("响应结果:成功！")
