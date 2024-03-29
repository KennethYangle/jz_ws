#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import os
import json
import rospy, rospkg
from PX4MavCtrlV4 import EarthModel
from geometry_msgs.msg import TwistStamped, Quaternion
from swarm_msgs.msg import Action
from mavros_msgs.msg import State, PositionTarget, AttitudeTarget, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_matrix, euler_from_matrix, quaternion_from_euler


# 无人机控制类
class Px4Controller:
    def __init__(self, drone_id, scene="32s"):
        self.arm_state = False
        self.offboard_state = False
        self.state = None
        self.command = TwistStamped()
        self.start_point = PoseStamped()
        self.start_point.pose.position.z = 4
        self.drone_id = drone_id
        self.drone_name = "drone_{}".format(self.drone_id)
        self.scene = scene
        self.rate = rospy.Rate(20)

        self.is_initialize_pos = False
        self.mav_yaw = 0
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_vel = np.array([0., 0., 0.])
        self.mav_R = np.identity(3)
        self.uavPosGPSHome = [28.1405220, 112.9825003, 53.220]

        # mavros topics
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        self.global_home_sub = rospy.Subscriber("/mavros/home_position/home", HomePosition, self.mav_home_cb)
        self.local_vel_pub =  rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.local_att_pub =  rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)                     # 发布速度指令（重要）
        self.pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)                        # 发布位置指令（初始化飞到厂房前使用）
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)                                              # 解锁服务
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)                                             # 飞行模式服务
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)                                          # 起飞服务
        print("Px4 Controller Initialized with {}".format(self.drone_name))

    # 任务开始前的一些动作，包括解锁、进offboard、飞到厂房前
    def start(self):
        for _ in range(2):
            self.vel_pub.publish(self.command)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            self.rate.sleep()

        # 不同场景做不同的准备动作
        if self.scene == "32s":             # 主要
            self.takeoff(h=1.8)

        if self.scene == "paper":           # 论文使用
            self.takeoff(h=6)
            des_pos = np.array([0, 35, 3.5])
            dis = np.linalg.norm(des_pos-self.mav_pos)
            command_vel = TwistStamped()
            while dis > 0.5:
                norm_vel = (des_pos-self.mav_pos)/dis*11
                command_vel.twist.linear.x,command_vel.twist.linear.y,command_vel.twist.linear.z = norm_vel
                self.vel_pub.publish(command_vel)
                dis = np.linalg.norm(des_pos-self.mav_pos)
                self.rate.sleep()

        if self.scene == "jz":           # 拒止项目
            # takeoff_pos=[0, 400, 50]    # ENU
            # takeoff_pos=[1305, 4933, 97]    # ENU
            takeoff_pos=[-1305, 1833, 97]    # ENU
            self.geo = EarthModel()
            lla = self.geo.enu2lla(takeoff_pos, self.uavPosGPSHome)
            lat, lon, alt = lla[0], lla[1], lla[2]

            print("latitude={}, longitude={}, altitude={}".format(lat, lon, alt))
            self.takeoffService(latitude=lat, longitude=lon, altitude=alt)
            rospy.loginfo("Taking off")
            time.sleep(0.5)

            # dis = np.linalg.norm(takeoff_pos - self.mav_pos)
            while abs(self.mav_pos[1] - takeoff_pos[1]) > 50:
                print("起飞中...")
                print(self.mav_pos)
                # self.takeoffService(latitude=lat, longitude=lon, altitude=alt)
                time.sleep(0.5)
                # dis = np.linalg.norm(takeoff_pos - self.mav_pos)

            # 保持前飞一段
            for i in range(20):
                self.offboard_state = self.offboard()
                # print("开始进入Offboard模式")

                q = Quaternion()
                q.w = 1.
                q.x = 0.
                self.moveByAttitudeThrust(q, 0.75)
                print("moveByAttitudeThrust")
                # self.moveByBodyRateThrust(0, -0.1, 0, 0.75)  # 可行值：(0.5, 0, 0, 0.75)
                # print("moveByBodyRateThrust")
                self.rate.sleep()

            # 盘旋
            
            # 盘旋：目标位置
            target_pos = np.array([-1791.3, 2201.8, 17.85])
            circle_R = np.linalg.norm(self.mav_pos[:2] - target_pos[:2])
            circle_H = self.mav_pos[2]
            circle_V = np.linalg.norm(self.mav_vel)
            # 盘旋：微分平坦参数
            Kp = 0.05
            Kv = 0.2

            for i in range(2800):
                # 微分平坦跟踪圆
                vec_p = self.mav_pos[:2] - target_pos[:2]
                vec_unit_p = vec_p / np.linalg.norm(vec_p)
                mav_pos_d = target_pos + np.array([vec_unit_p[0] * circle_R, vec_unit_p[1] * circle_R, circle_H])
                # print("mav_pos_d: {}, mav_pos: {}".format(mav_pos_d, self.mav_pos))

                mav_vel_d = circle_V * np.array([-vec_unit_p[1], vec_unit_p[0], 0.])
                if mav_vel_d.dot(self.mav_vel) < 0:
                    mav_vel_d = -mav_vel_d
                # print("mav_vel_d: {}, mav_vel: {}".format(mav_vel_d, self.mav_vel))

                an_norm = circle_V * circle_V / circle_R
                an = np.array([-vec_unit_p[0] * an_norm, -vec_unit_p[1] * an_norm, 0.])
                a_d = an + Kv * (mav_vel_d - self.mav_vel) + Kp * (mav_pos_d - self.mav_pos)
                # print("an: {}, a_d: {}".format(an, a_d))

                # 微分平坦控制
                g = np.array([0, 0, 9.8])
                V = np.linalg.norm(self.mav_vel)
                r1 = self.mav_vel / V
                a_w1 = r1.dot(a_d - g)      # 标量
                a_n = a_d - g - a_w1 * r1   # 矢量
                a_w3 = -np.linalg.norm(a_n) # 标量
                r3 = a_n / a_w3
                r2 = np.cross(r3, r1)
                R = np.array([r1 ,r2, r3]).T
                M = np.identity(4)
                M[:3,:3] = R
                # q_array = quaternion_from_matrix(M)
                euler = euler_from_matrix(M)
                q_array = quaternion_from_euler(-euler[0], -euler[1], euler[2])

                q = Quaternion()
                q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
                thrust = 0.75
                self.moveByAttitudeThrust(q, thrust)
                self.rate.sleep()



        if self.scene == "freeflight":      # test for freeflight.py
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

        if self.scene == "attack_in_oldfactory":      # oldfactory场景起点（以后删）
            self.start_point.pose.position.x = 1
            self.start_point.pose.position.y = -12
            self.start_point.pose.position.z = 2.5
            for _ in range(300):
                self.pos_pub.publish(self.start_point)
                self.rate.sleep()

        if self.scene == "freeflight":      # test for tube
            des_pos = np.array([46.55, 36.75, 1.0])
            self.start_point.pose.position.x = des_pos[0]
            self.start_point.pose.position.y = des_pos[1]
            self.start_point.pose.position.z = des_pos[2]
            dis = np.linalg.norm(des_pos-self.mav_pos)
            while dis > 0.5:
                self.pos_pub.publish(self.start_point)
                dis = np.linalg.norm(des_pos-self.mav_pos)
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

    # 无人机位置速度回调函数
    def local_vel_callback(self, msg):
        self.is_initialize_vel = True
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    
    def mav_home_cb(self, msg):
        self.uavPosGPSHome = [msg.geo.latitude, msg.geo.longitude, msg.geo.altitude]


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
            print("Vechile Offboard Mode")
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
            # command = TwistStamped()
            # command.twist.linear.z = vz
            # self.vel_pub.publish(command)
            rospy.sleep(0.05)
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = (abs(self.mav_pos[2]- h) < 0.05)

    def moveByPosENU(self, E=None, N=None, U=None, mav_yaw=None):
        mav_yaw = mav_yaw if mav_yaw is not None else self.mav_yaw
        E = E if E is not None else self.mav_pos[0]
        N = N if N is not None else self.mav_pos[1]
        U = U if U is not None else self.mav_pos[2]
        command_vel = construct_postarget_ENU(E, N, U, mav_yaw)
        self.local_vel_pub.publish(command_vel)

    def moveByAttitudeThrust(self, q, thrust):
        '''
        q: geometry_msgs/Quaternion type
        thrust: float 0-1
        body frame: FLU, 不能控偏航, 四元数网站: https://quaternions.online/
        '''
        target_raw_attitude = AttitudeTarget()
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ROLL_RATE + AttitudeTarget.IGNORE_PITCH_RATE + AttitudeTarget.IGNORE_YAW_RATE

        target_raw_attitude.orientation = q
        target_raw_attitude.thrust = thrust
        self.local_att_pub.publish(target_raw_attitude)

    def moveByBodyRateThrust(self, roll_rate, pitch_rate, yaw_rate, thrust):
        '''
        body frame: FLU, 不能控偏航
        '''
        target_raw_attitude = AttitudeTarget()
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE

        target_raw_attitude.body_rate.x = roll_rate
        target_raw_attitude.body_rate.y = pitch_rate
        target_raw_attitude.body_rate.z = yaw_rate
        target_raw_attitude.thrust = thrust
        self.local_att_pub.publish(target_raw_attitude)
    
    
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


class Assemble:

    def __init__(self, param_id, px4):
        self.px4 = px4
        self.start_time = time.time()
        
        self.pipeline_cmd = TwistStamped()
        self.dj_cmd = TwistStamped()
        self.obs_cmd = TwistStamped()
        self.dj_action = Action()

        self.Pipeline_cmd_sub = rospy.Subscriber('Pipeline_cmd', TwistStamped, self.Pipeline_cmd_callback)
        self.DJ_cmd_sub = rospy.Subscriber('DJ_cmd', AttitudeTarget, self.DJ_cmd_callback)
        self.Obs_cmd_sub = rospy.Subscriber('Obs_cmd', TwistStamped, self.Obs_cmd_callback)
        self.Expect_action_sub = rospy.Subscriber('expect_action'+str(param_id), Action, self.Expect_action_callback)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    def Pipeline_cmd_callback(self, msg):
        self.pipeline_cmd = msg

    def DJ_cmd_callback(self, msg):
        self.dj_cmd = msg

    def Obs_cmd_callback(self, msg):
        self.obs_cmd = msg

    def Expect_action_callback(self, msg):
        self.dj_action = msg


    def begin_task(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.dj_action.dj == True:
                # self.vel_pub.publish(self.dj_cmd)
                self.px4.moveByAttitudeThrust(self.dj_cmd.orientation, self.dj_cmd.thrust)
            else:
                self.vel_pub.publish(self.pipeline_cmd)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("assemble", anonymous=True)
    param_id = rospy.get_param("~drone_id")

    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)

    # 飞机初始化，解锁、offboard、飞到厂房前
    px4 = Px4Controller(param_id, setting["SCENE"])
    ass = Assemble(param_id, px4)
    px4.start()
    ass.begin_task()