#!/usr/bin/env python
# coding=utf-8

# 本程序为决策模块主要实现。负责调度任务全流程，从剧本中拿到管道并与集群管道控制器交互

import rospy
import os, sys
import json
import threading
import numpy as np

from swarm_msgs.msg import Pipeline, Pipeunit
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from std_msgs.msg import UInt64


# 无人机控制类
class Px4Controller:
    def __init__(self, drone_name):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.start_point.pose.position.z = 4
        self.drone_name = drone_name
        self.rate = rospy.Rate(20)
        # mavros topics
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)                     # 发布速度指令（重要）
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)                        # 发布位置指令（初始化飞到厂房前使用）
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)                                              # 解锁服务
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)                                             # 飞行模式服务
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)                                          # 起飞服务
        print("Px4 Controller Initialized with {}".format(drone_name))

    # 任务开始前的一些动作，包括解锁、进offboard、飞到厂房前
    def start(self):
        for _ in range(10):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.rate.sleep()

        for _ in range(100):
            self.pos_pub.publish(self.start_point)
            self.rate.sleep()

        self.start_point.pose.position.x = 10.5
        self.start_point.pose.position.y = -7
        self.start_point.pose.position.z = 2
        for _ in range(300):
            self.pos_pub.publish(self.start_point)
            self.rate.sleep()

    # 悬停
    def idle(self):
        print("I'm in idle state!")
        idle_cmd = TwistStamped()
        while not rospy.is_shutdown():
            self.vel_pub.publish(idle_cmd)
            self.rate.sleep()

    # 解锁
    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    # 上锁
    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    # 进offboard模式
    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    # 起飞
    def takeoff(self):
        if self.takeoffService(altitude=3):
            return True
        else:
            print("Vechile Takeoff failed")
            return False

def spin():
    rospy.spin()


if __name__ == '__main__':
    # 载入剧本文件
    # file_path = os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play_2directions_12drones.json")
    # play_file = open(file_path)
    # play = json.load(play_file)
    # # print(json.dumps(play))

    # ROS初始化，从launch文件获取参数
    rospy.init_node('decision_node', anonymous=True)
    spin_thread = threading.Thread(target=spin)
    spin_thread.start()
    param_id = rospy.get_param("~drone_id")
    drone_name = "drone_{}".format(param_id)

    # # 判断是否有对应的剧本
    # if drone_name not in play.keys():
    #     print("Name Error! {} not in {}".format(drone_name, file_path))
    #     sys.exit(1)
    

    # 飞机初始化，解锁、offboard、飞到厂房前
    px4 = Px4Controller(drone_name)
    px4.start()


    # 待添加 -----------------------------------------
    # 绕飞、集群飞行的路径点发布

    rospy.spin()
