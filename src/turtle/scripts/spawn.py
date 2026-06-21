#!/usr/bin/env python3
# _*_ coding: utf-8 _*

import rospy
import sys
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse

def main():
    rospy.init_node('spawn_node')
    client = rospy.ServiceProxy('/spawn', Spawn)    
    if len(sys.argv) != 5:
        rospy.logerr("请正确提交参数")
        sys.exit(1) 
    client.wait_for_service()
    req = SpawnRequest()
    req.x = float(sys.argv[1])
    req.y = float(sys.argv[2])
    req.theta = float(sys.argv[3])
    req.name = sys.argv[4]
    resp = client.call(req)

if __name__ == '__main__':
    main()
    rospy.loginfo("服务调用成功")
