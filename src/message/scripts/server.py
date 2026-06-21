#! /usr/bin/env python3
# _*_ coding: utf-8 _*


import rospy
from message.srv import text,textRequest,textResponse

def res(req):
    num = req.num1 + req.num2
    result=req.age
    rospy.loginfo("提交的数据:num1 = %d, num2 = %d, 结果:num = %d, 年龄= %f",req.num1, req.num2, num, result)
    res = textResponse()
    res.num = num
    res.result = result
    return res

def server():
    rospy.init_node("server_node")
    server = rospy.Service("text",text,res)         

if __name__ == "__main__":
    server()
    rospy.spin()
