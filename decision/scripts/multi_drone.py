#!/usr/bin/env python
# coding=utf-8

# 本程序为决策模块主要实现。负责调度任务全流程，从剧本中拿到管道并与集群管道控制器交互

import rospy
import os, sys
import json
import threading
import numpy as np

from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from std_msgs.msg import UInt64


# 无人机控制类
class Px4Controller:
    def __init__(self, drone_id):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.start_point.pose.position.z = 4
        self.drone_id = drone_id
        self.drone_name = "drone_{}".format(self.drone_id)
        self.rate = rospy.Rate(20)

        self.is_initialize_pos = False
        self.mav_yaw = 0
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_R = np.identity(3)

        # mavros topics
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_pub =  rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)                     # 发布速度指令（重要）
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)                        # 发布位置指令（初始化飞到厂房前使用）
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)                                              # 解锁服务
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)                                             # 飞行模式服务
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)                                          # 起飞服务
        print("Px4 Controller Initialized with {}".format(self.drone_name))

    # 任务开始前的一些动作，包括解锁、进offboard、飞到厂房前
    def start(self):
        for _ in range(10):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.rate.sleep()

        self.takeoff()

        if self.drone_id == 1:
            self.start_point.pose.position.x = 0
            self.start_point.pose.position.y = 0
            self.start_point.pose.position.z = 2
        elif self.drone_id == 2:
            self.start_point.pose.position.x = 4
            self.start_point.pose.position.y = 0
            self.start_point.pose.position.z = 2
        elif self.drone_id == 3:
            self.start_point.pose.position.x = 0
            self.start_point.pose.position.y = 4
            self.start_point.pose.position.z = 2
        for _ in range(300):
            self.pos_pub.publish(self.start_point)
            self.rate.sleep()

    # 无人机位置姿态回调函数
    def local_pose_callback(self, msg):
        if not self.is_initialize_pos:
            self.mav_yaw_0 = self.mav_yaw
        self.is_initialize_pos = True
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
        self.mav_R = np.array([
            [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
        ])

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
    def takeoff(self, vz=0.8,h=1.8):
        self.mav_yaw_0 = self.mav_yaw

        self.moveByPosENU(U=h)
        takeoff_done = False
        while not takeoff_done:
            self.moveByPosENU(U=h)
            rospy.sleep(0.05)
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = (abs(self.mav_pos[2]- h) < 0.2)

    def moveByPosENU(self, E=None, N=None, U=None, mav_yaw=None):
        mav_yaw = mav_yaw if mav_yaw is not None else self.mav_yaw
        E = E if E is not None else self.mav_pos[0]
        N = N if N is not None else self.mav_pos[1]
        U = U if U is not None else self.mav_pos[2]
        command_vel = construct_postarget_ENU(E, N, U, mav_yaw)
        self.local_vel_pub.publish(command_vel)
    
    
def construct_postarget_ENU(E=0, N=0, U=0, yaw=0, yaw_rate = 0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()
    
    '''
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9

    MAV_FRAME_BODY_FRD
    '''

    target_raw_pose.coordinate_frame = 1

    target_raw_pose.position.x = E
    target_raw_pose.position.y = N
    target_raw_pose.position.z = U

    target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE

    target_raw_pose.yaw = yaw
    target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose


if __name__ == '__main__':
    # 载入剧本文件
    # file_path = os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play_2directions_12drones.json")
    # play_file = open(file_path)
    # play = json.load(play_file)
    # # print(json.dumps(play))

    # ROS初始化，从launch文件获取参数
    rospy.init_node('decision_node', anonymous=True)
    drone_id = int(rospy.get_param("~drone_id"))

    # # 判断是否有对应的剧本
    # if drone_name not in play.keys():
    #     print("Name Error! {} not in {}".format(drone_name, file_path))
    #     sys.exit(1)
    

    # 飞机初始化，解锁、offboard、飞到厂房前
    px4 = Px4Controller(drone_id)
    px4.start()


    # 待添加 -----------------------------------------
    # 绕飞、集群飞行的路径点发布

