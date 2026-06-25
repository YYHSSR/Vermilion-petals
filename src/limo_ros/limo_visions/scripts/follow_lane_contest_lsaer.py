#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
from math import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

global out,pose_x,pose_y,front_dist,left_dist,right_dist,red_x,red_y,red_area
out = 1
pose_x = pose_y= 0
front_dist = left_dist = right_dist = 999.0
#red_x：图像横向像素坐标，范围 0 - 640
#red_y：图像纵向像素坐标，范围 0 - 480
#red_area：红色标志点面积，范围 0 - 307200
red_x = red_y = -1
red_area = 0.0

class follow_lane:
    def __init__(self):
        #订阅道路线位置
        self.Pose_sub = rospy.Subscriber("/lane_detect_pose", Pose, self.velctory)
        #订阅雷达数据
        self.Scan_sub = rospy.Subscriber("/limo/scan", LaserScan, self.scan,queue_size=5)
        #订阅地图坐标 /pose
        self.map_pose = rospy.Subscriber("/pose", PoseStamped, self.pose,queue_size=3)
        #订阅红点位置
        self.Red_sub = rospy.Subscriber("/red_detect_pose", Pose, self.red_pose, queue_size=1)
        self.is_stopping = False
        self.is_approaching = False
        self.target_stop_dist = 0.0
        self.cooldown_until = rospy.Time(0)
        #发布速度指令
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        # 注册节点关闭时的回调函数，确保退出时车辆停止
        self.found_line = False
        rospy.on_shutdown(self.clean_shutdown)

    #节点关闭回调函数
    def clean_shutdown(self):
        rospy.loginfo("Shutdown hook triggered. Stopping the robot...")
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)
    
    #获取地图坐标
    def pose(self,PoseStamped):
        global pose_x,pose_y
        pose_x = PoseStamped.pose.position.x
        pose_y = PoseStamped.pose.position.y

    #获取红点坐标
    def red_pose(self, msg):
        global red_x, red_y, red_area
        red_x = msg.position.x
        red_y = msg.position.y
        red_area = msg.position.z

    #获取雷达数据
    def scan(self,msg):
        global front_dist,left_dist,right_dist
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 999
        ranges[np.isnan(ranges)] = 999
        # 计算每个数据点对应的角度（角度，单位：度）
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        angles_deg = np.degrees(angles)
        # 过滤掉小于等于 0.07m 的雷达盲区数据（根据 Gazebo 配置，雷达盲区调整为 0.08m）
        valid_mask = (ranges > 0.07) & (ranges < 999)
        # 前方距离 (-15° 到 15°)
        front_idx = np.where((angles_deg >= -15) & (angles_deg <= 15) & valid_mask)[0]
        front_dist = float(np.min(ranges[front_idx])) if len(front_idx) > 0 else 999.0
        # 左侧距离 (45° 到 90°)
        left_idx = np.where((angles_deg >= 45) & (angles_deg <= 90) & valid_mask)[0]
        left_dist = float(np.min(ranges[left_idx])) if len(left_idx) > 0 else 999.0
        # 右侧距离 (-90° 到 -45°)
        right_idx = np.where((angles_deg >= -90) & (angles_deg <= -45) & valid_mask)[0]
        right_dist = float(np.min(ranges[right_idx])) if len(right_idx) > 0 else 999.0
        # 以 20Hz 的频率限流打印雷达数据
        # rospy.loginfo_throttle(0.05, "left: %.4f, front: %.4f, right: %.4f" % (left_dist, front_dist, right_dist))

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

    #出库
    def go(self):
        global out
        if out==1 :
            self.run_time(0.25,0,2.9)
            self.run_time(0.12,-0.7,15)
            out = 0

    #入库
    def back(self):
        global out
        self.run_time(0.05,-0.08,0.4)
        self.run_time(0.15,0,0.6)
        self.run_time(0.13,-0.387,5.2)
        self.run_time(0.035,0,1)
        self.run_time(0.13,0.78,4.3)
        self.run_time(0.05,0,0.85)

    def velctory(self,Pose):        
        x = Pose.position.x
        y = Pose.position.y
        z = Pose.position.z

        global out,pose_x,pose_y,front_dist,left_dist,right_dist,red_x,red_y,red_area
    
        #停车
        # 1. 如果当前正在执行停靠等待，直接发布停止速度并返回（防止多线程回调重入）
        if self.is_stopping:
            vel = Twist()
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.vel_pub.publish(vel)
            return

        # 2. 检查是否检测到红点，如果没在冷却期内且没有在逼近，触发逼近状态并记录初始雷达距离
        now = rospy.Time.now()
        if now >= self.cooldown_until and not self.is_approaching:
            # 只有当红点可见(red_area > 100)、距离合适(red_y >= 220)
            if red_area > 100 and red_y >= 220 :
                self.is_approaching = True
                self.target_stop_dist = front_dist - 0.50  # 设从小车检测到 red_y=220 到停在红圈正上方所需行驶的物理距离为 0.50 米
                rospy.loginfo("Red dot detected at y=%.2f. Using LiDAR relative stop: start_dist=%.4f, target_dist=%.4f", 
                              red_y, front_dist, self.target_stop_dist)

        # 3. 如果在逼近状态下，单独接管速度控制并最终停靠，但转向仍使用本回调传入的 x, y, z 计算出的巡线角速度
        if self.is_approaching:
            # 限制前进速度与制动触发
            dist_error = front_dist - self.target_stop_dist
            if dist_error <= 0.02:
                rospy.loginfo("Reached relative stop position (front_dist=%.4f, target=%.4f). Starting 5.5s park.", front_dist, self.target_stop_dist)
                self.is_stopping = True
                self.is_approaching = False
                self.run_time(0.0, 0.0, 5.5)
                self.is_stopping = False
                self.cooldown_until = rospy.Time.now() + rospy.Duration(5.0)
                rospy.loginfo("5.5s stop finished. Resuming lane following.")
                return
            else:
                # 比例减速，最低速度 0.05m/s，最高速度 0.15m/s
                lin_vel = np.clip(0.15 * dist_error, 0.05, 0.15)
            
            # 逼近期间计算并使用正常的寻线转向控制，不破坏原本的巡线转向计算规则
            if z <= 5:
                ang_vel = 0.0
            else:
                if x < 0:
                    if y <= 340:
                        ang_vel = -1.2
                    else:
                        ang_vel = 0.0
                else:
                    target_x = 140
                    error = target_x - x
                    ang_vel = error * 0.007

            # 设定转向速度范围
            max_ang_vel = 0.8
            min_ang_vel = -0.8
            if ang_vel >= max_ang_vel:
                ang_vel = max_ang_vel
            if ang_vel <= min_ang_vel:
                ang_vel = min_ang_vel

            vel = Twist()
            vel.linear.x = lin_vel
            vel.angular.z = ang_vel
            self.vel_pub.publish(vel)
            return

        #self.go()
        #巡线逻辑  
        # 如果尚未首次检测到车道线，检查是否现在检测到了
        if not self.found_line:
            if x >= 0:
                self.found_line = True
                rospy.loginfo("Yellow lane line successfully detected for the first time. Starting follow control!")
            else:
                # 尚未检测到线，保持静止，避免启动瞬间因为没有画面而产生错误转向
                vel = Twist()
                vel.linear.x = 0.0
                vel.angular.z = 0.0
                self.vel_pub.publish(vel)
                return

        vel = Twist()
        #设置速度范围
        max_ang_vel = 0.8
        min_ang_vel = -0.8
        #当检测不到道路线时，停止
        if z <= 5 :
            lin_vel = 0
            ang_vel = 0
            vel.linear.x  = lin_vel
            vel.angular.z = ang_vel
            self.vel_pub.publish(vel)
            rospy.sleep(1)
            #发布速度打印
            # rospy.loginfo(
            #         "Publsh velocity command[{} m/s, {} rad/s]".format(
            #             vel.linear.x, vel.angular.z))
        else  : 
            # 如果黄色车道线丢失 (x < 0)
            if x < 0:
                # 检查横向车道线以决定是否执行右转弯寻找车道
                if y <= 340:
                    lin_vel = 0.19
                    ang_vel = -1.2
                else:
                    lin_vel = 0.19
                    ang_vel = 0.0
            else:
                # 正常检测到黄色线，进行比例控制以保持在车道中间（目标列为 60）
                target_x = 140
                lin_vel = 0.19
                error = target_x - x
                ang_vel = error * 0.007
                
            # 设定转向速度范围
        if ang_vel >= max_ang_vel:
            ang_vel = max_ang_vel
        if ang_vel <= min_ang_vel:
            ang_vel = min_ang_vel
        #发布速度指令
        vel.linear.x  = lin_vel
        vel.angular.z = ang_vel
        self.vel_pub.publish(vel)

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("follow_lane", anonymous=False)
        rospy.loginfo("Starting follow lane")
        follow_lane()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down follow_object node.")
        cv2.destroyAllWindows()