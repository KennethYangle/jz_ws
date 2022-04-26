#!/usr/bin/env python
# coding=utf-8

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
# from std_msgs import Float64  Float64MultiArray
from std_msgs.msg import Float64
import time
import math
import numpy as np

# keyboard input
import threading
import Tkinter

from rflysim_ros_pkg.msg import Obj
from optical_flow.msg import index


# bs ----------njq_bs.bat-----------  UE4_MAP = VisionRingBlank

# Pos_Tong = np.array([[0.5, 5,  2],
#                     [1.2, 10, 2],
#                     [-1.8, 8, 2],  # 2
#                     [5, 6, 2],
#                     [2.2, 18, 2],
#                     [5, 20, 2],
#                     [10, 22, 2],
#                     [5, 17,  2],
#                     [-1.5, 12,  2],
#                     [0.5, 15, 2],  # 2.1   [0, 15, 2],
#                     [-1.5, 19,  2],
#                     [5, 8, 2],
#                     [5, 25, 2],
#                     [0, 22,  2],
#                     [3.5, 24,  2],
#                     [-2, 25, 2],
#                     [5, 13,  2]])
# Pos_Tong = np.array([[0.5, 5,  0],
#                     [1.2, 10, 0],
#                     [-1.8, 8, 0],  # 2
#                     [5, 6, 0],
#                     [2.2, 18, 0],
#                     [5, 20, 0],
#                     [10, 22, 0],
#                     [5, 17,  0],
#                     [-1.5, 12,  0],
#                     [0.5, 15, 0],  # 2.1   [0, 15, 2],
#                     [-1.5, 19,  0],
#                     [5, 8, 0],
#                     [5, 25, 0],
#                     [0, 22,  0],
#                     [3.5, 24, 0],
#                     [-2, 25, 0],
#                     [5, 13,  0]])
# Tree
Pos_Tong = np.array([[0.5, 5,  0],
                    [1.2, 10, 0],
                    [-1.8, 8, 0],  # 2
                    [5, 6, 0],
                    [2.2, 18, 0],
                    [5, 20, 0],
                    [10, 22, 0],
                    [6, 17,  0],
                    [-0.5, 12,  0],
                    [0.5, 15, 0],  # 2.1   [0, 15, 2],
                    [-1.5, 19,  0],
                    [3, 8, 0],
                    [5, 25, 0],
                    [0, 22,  0],
                    [3.5, 24, 0],
                    [-2, 25, 0],
                    [5, 13,  0],
                    [-3, 21, 0],
                    [-2, 17, 0],
                    [-3.5, 11, 0],
                    [-2.5, 3, 0],
                    [6.5, 20, 0],
                    [8, 15, 0],
                    [7, 10, 0],
                    [7.5, 4, 0]])  # 01

ID_Tong = [20, 30, 40, 50, 60, 70, 80, 90, 91, 92, 93, 94, 95,
           96, 97, 98, 99, 101, 102, 103, 104, 105, 106, 107, 108]

# Pos_Sphere = np.array([[1.5, 10,  2],
#                        [-0.5, 6,  2],
#                        [-1, 24,  2],
#                        [-3, 8,  3],
#                        [-2.5, 18,  2.5],
#                        [-3.5, 30,  2.2],
#                        [-1, 24,  3],
#                        [2.2, 15, 2]])
# ID_Sphere = [101, 102, 103, 104, 105, 106, 107, 108]


class Px4Controller:

    def __init__(self):
        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.local_enu_position = None

        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.imu_rate = np.array([0, 0, 0])

        self.state = None
        self.mavros_state = State()
        self.command = TwistStamped()
        self.mav_pos = np.array([0, 0, 0])
        self.mav_vel = np.array([0, 0, 0])
        self.cmd_vel = np.array([0, 0, 0])
        self.cmd_body_vel = np.array([0, 0, 0])
        self.cmd_force_vel = np.array([0, 0, 0])
        self.ttc = 0
        self.cmd_pos_vel = np.array([0, 0, 0])
        # self.q = Queue()
        self.mav_yaw = 0
        self.cmd_yaw = 0
        self.mav_roll = 0
        self.mav_pitch = 0
        self.mav_R = np.zeros((3, 3))
        # self.desire_pos = np.array([200, -15, 2.5])  # [20, 30, 0]
        # self.desire_pos = np.array([0, 30, 0])  # [20, 30, 0]
        # OldFactory desire pos
        self.sta_pos = np.array([[2, 0, 0],
                                 [4, 0, 0],
                                 [6, 0, 0],
                                 [0, 0, 0],
                                 [-2, 0, 0]])
        self.start_pos = self.sta_pos[3]
        self.final = np.array([[2, 30, 0],
                               [4, 30, 0],
                               [6, 30, 0],
                               [0, 30, 0],
                               [-2, 30, 0]])
        self.final_pos = self.final[4]
        self.desire_pos = np.array([0, 0, 0])  # [20, 30, 0]
        self.start_yaw = 0

        self.R_cb = np.array([[1, 0, 0],
                             [0, 0, 1],
                             [0, -1, 0]])
        self.n_cc = np.array([0, 0, 1])

        self.vd = np.array([0, 0, 0], dtype=np.float64)
        self.ko = 2.5  # the angular rate to avoid obstacle

        self.kp_vd = 2.0  # the p_control about desire velicoty

        self.kvod = 2.5  # the p_control about desire velicoty by matrix initial value:1   2.5

        self.WIDTH = 720  # simulation 720
        self.HEIGHT = 405  # simulation 405

        self.w, self.h = self.WIDTH, self.HEIGHT
        self.u0 = self.w/2
        self.v0 = self.h/2
        self.x0 = self.u0
        self.y0 = self.v0

        # 346.6  # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2),不同仿真环境以及真机实验中需要依据实际情况进行修改
        self.f = 632
        self.pos_i = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.ttc_dis = 0
        self.of_dis = 0
        self.all_dis = 0

        self.sum_v_ttc = [0.0, 0.0, 0.0]
        self.sum_v_of = [0.0, 0.0, 0.0]

        '''
        ros subscribers   /balance_force  /ttc_mean
        '''
        self.local_pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber(
            "/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber(
            "/mavros/imu/data", Imu, self.imu_callback)
        self.balance_force = rospy.Subscriber(
            "/balance_force", Float64, self.force_callback)
        self.ttc_mean = rospy.Subscriber(
            "/ttc_mean", Float64, self.ttc_callback)
        self.ttc_mssages_sub = rospy.Subscriber(
            "/TTC_message", index, self.ttc_mssages_cb)
        self.of_mssages_sub = rospy.Subscriber(
            "/OF_message", index, self.of_mssages_cb)
        self.mav_sub = rospy.Subscriber(
            "mavros/local_position/velocity_local", TwistStamped, self.mav_vel_cb)
        # self.imu_sub = rospy.Subscriber("mavros/imu/data", Imu, self.mav_vel_cb)

        self.is_takeoff, self.is_hover, self.is_Forward, self.is_land = False, False, False, False
        self.is_force = False
        self.is_offboard = False

        self.P_pos = 0.1
        self.P_yaw = 0.1
        self.P_heigh = 0.5
        self.start_height = 0
        self.distance = 0

        '''
        ros publishers
        '''
        self.vel_pub = rospy.Publisher(
            'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.sphere_pub = rospy.Publisher(
            'ue4_ros/obj', Obj, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy(
            '/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")

    def call(self, event):
        k = event.keysym
        if k == "p":
            self.is_takeoff = True
            self.is_hover = False
            self.is_Forward = False
            self.is_land = False
            self.is_force = False
        elif k == "b":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = True
            self.is_land = False
            self.is_force = False
        elif k == "c":
            self.is_takeoff = False
            self.is_hover = True
            self.is_Forward = False
            self.is_land = False
            self.is_force = False
        elif k == "d":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = False
            self.is_land = True
            self.is_force = False
        elif k == "a":
            self.is_takeoff = False
            self.is_hover = False
            self.is_Forward = False
            self.is_land = False
            self.is_force = True
        elif k == "o":
            self.is_offboard = True
        time.sleep(0.02)

    def read_kbd_input(self):
        win = Tkinter.Tk()
        frame = Tkinter.Frame(win, width=200, height=120)
        frame.bind("<Key>", self.call)
        frame.focus_set()
        frame.pack()
        win.mainloop()

    def start(self):
        rospy.init_node("offboard_node")
        rate = rospy.Rate(20)

        while(not self.mavros_state.connected):
            print(self.mavros_state.connected)
            rate.sleep()

        for i in range(10):
            # self.sphere_control()
            self.vel_pub.publish(self.command)
            # self.arm_state = self.arm()
            # self.offboard_state = self.offboard()
            rate.sleep()

        self.offboard_state = SetMode()
        self.offboard_state.custom_mode = "OFFBOARD"
        self.arm_state = CommandBool()
        self.arm_state.value = True
        # self.sphere_control()

        start_time = rospy.Time.now()
        '''
        main ROS thread
        '''

        cnt = -1
        while (rospy.is_shutdown() is False):
            print("time: {}".format(
                rospy.Time.now().to_sec() - start_time.to_sec()))
            self.sphere_control()

            cnt += 1
            if self.is_offboard == False:
                if self.mavros_state.mode == "OFFBOARD":
                    # self.flightModeService.call(custom_mode='POSCTL')
                    resp1 = self.flightModeService(0, "POSCTL")
                if cnt % 10 == 0:
                    print("Enter MANUAL mode")
                rate.sleep()
                self.start_height = self.mav_pos[2]
                self.start_yaw = self.mav_yaw
                continue
            else:
                if self.mavros_state.mode != "OFFBOARD":
                    # self.flightModeService.call(custom_mode='OFFBOARD')
                    resp1 = self.flightModeService(
                        0, self.offboard_state.custom_mode)
                    if resp1.mode_sent:
                        print("Offboard enabled")
                    # start_time = rospy.Time.now()
                self.mav_state()
            # start_time = rospy.Time.now()
            if self.is_force == True:
                cmd_flow = self.obs_control()
                # cmd_flow = self.cmd_force_vel
                kpa = 1

                if self.all_dis > 0.02 and self.all_dis < 0.1:
                    kpa = 0.4  # 0.5
                elif self.all_dis > 0.1:
                    kpa = 0
                else:
                    kpa = 1  # 1

                if self.mav_pos[1] > 25:
                    cmd_flow[0] = 0
                    cmd_flow[1] = 0
                    cmd_flow[2] = 0
                    kpa = 1.5
                self.command.twist.linear.x = kpa * \
                    self.cmd_vel[0] + cmd_flow[0]
                self.command.twist.linear.y = self.cmd_vel[1] + cmd_flow[1]
                self.command.twist.linear.z = self.cmd_vel[2] + cmd_flow[2]
                self.command.twist.angular.z = 0  # self.cmd_yaw
            else:
                self.command.twist.linear.x = self.cmd_vel[0]
                self.command.twist.linear.y = self.cmd_vel[1]
                self.command.twist.linear.z = self.cmd_vel[2]
                self.command.twist.angular.z = 0  # self.cmd_yaw
            # balance
            # self.command.twist.linear.x = self.cmd_vel[0]
            # self.command.twist.linear.y = self.cmd_vel[1]
            # self.command.twist.linear.z = self.cmd_vel[2]
            # self.command.twist.angular.z = self.cmd_yaw
            self.vel_pub.publish(self.command)
            # print("TTC_mean: {}".format([self.ttc]))
            if self.is_force == True:
                print("mav_pos: {}".format(
                    [self.mav_pos[0], self.mav_pos[1], self.mav_pos[2]]))
                print("all_dis:{}".format(self.all_dis))
                print("cmd_flow:{}".format(cmd_flow))
                print("cmd_vel:{}".format(
                    [self.cmd_vel[0], self.cmd_vel[1], self.cmd_vel[2]]))
            rate.sleep()

    def mav_state(self):
        self.pos_vel()
        self.control_yaw()
        # cmd = self.cmd_pos_vel + self.cmd_force_velc
        cmd = self.cmd_force_vel*2.0

        if self.mav_pos[1] > 20:
            cmd = self.SatIntegral(cmd, 1.5, -1.5)
        else:
            cmd = self.SatIntegral(cmd, 1.5, -1.5)

        if self.is_takeoff == True:
            self.cmd_body_vel = np.array([0, 0, 1])
            self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            self.start_height = self.mav_pos[2]
            self.cmd_yaw = 0
        elif self.is_Forward == True:
            self.cmd_body_vel = np.array([2, 0, 0])
            self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            self.start_height = self.mav_pos[2]
            self.cmd_vel[2] = 0
            # self.cmd_vel = np.array([0, 2, 0])
            self.cmd_yaw = 0
            # self.cmd_vel = [0, 1, 0]
        elif self.is_land == True:
            # self.cmd_body_vel = np.array([0, 0, -1])
            # self.cmd_vel = self.mav_R.dot(self.cmd_body_vel)
            self.cmd_vel = np.array([0, 0, -1])
            self.cmd_vel[0] = 0
            self.cmd_vel[1] = 0
            self.start_height = self.mav_pos[2]
            self.cmd_yaw = 0
        elif self.is_force == True:
            # self.sphere_control()
            # self.cmd_vel = np.array([0, 1.5, 0])  # 1.0
            # self.cmd_vel = self.mav_R.dot(self.cmd_vel)
            # if(np.linalg.norm(self.cmd_pos_vel) < 0.5):
            #     cmd[0] = 0
            #     cmd[1] = 0
            #     cmd[2] = 0
            # self.cmd_vel = cmd + self.cmd_pos_vel
            self.desire_pos = self.final_pos
            self.cmd_vel = self.cmd_pos_vel
        elif self.is_hover == True:
            self.desire_pos = self.start_pos
            self.cmd_vel = self.cmd_pos_vel
        else:
            self.cmd_vel = np.array([0, 0, 0])
            self.start_height = self.mav_pos[2]
            self.cmd_yaw = 0

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
        # self.mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        self.mav_pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_yaw = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
        self.mav_pitch = math.asin(2*(q0*q2 - q1*q3))
        self.mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
        R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                         [2*(q1*q2+q0*q3), q0**2-q1**2 +
                             q2**2-q3**2, 2*(q2*q3-q0*q1)],
                         [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
        R_ba = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        self.mav_R = R_ae.dot(R_ba)  # 机体坐标系  x————向右  y————向前   z————向上

    def force_callback(self, msg):
        # dlt_x = (self.desire_pos[0] - self.mav_pos[0]) * \
        #     (self.desire_pos[0] - self.mav_pos[0])
        # dlt_y = (self.desire_pos[1] - self.mav_pos[1]) * \
        #     (self.desire_pos[1] - self.mav_pos[1])
        # self.distance = math.sqrt(dlt_x + dlt_y)
        self.cmd_force_vel = np.array([0, 0, 0])
        # self.cmd_force_vel[0] = 0.07 * msg.data  # 0.1
        # if(self.ttc > 10):
        #     msg.data = 0
        self.cmd_force_vel[0] = 0.07 * msg.data  # 0.1
        # print("cmd_force: {}".format(self.cmd_force_vel[0]))
        self.cmd_force_vel = self.mav_R.dot(self.cmd_force_vel)  # body to NEU
        self.cmd_force_vel[2] = 0
        # self.cmd_force_vel[2] = 0

    def ttc_callback(self, msg):
        self.ttc = msg.data   #
        # print("TTC_mean: {}".format(np.array([self.ttc])))

    def ttc_mssages_cb(self, msg):
        # n_co = np.array([ttc_msg[0] - self.u0, ttc_msg[1] -
        #                 self.v0, self.f], dtype=np.float64)
        # n_co /= np.linalg.norm(n_co)
        self.ttc_dis = 0
        self.ttc_msg = []
        msg_data = msg.data
        cal_ttc = [0.0, 0.0, 0.0]
        self.sum_v_ttc = [0.0, 0.0, 0.0]

        for i in range(len(msg_data)):
            self.ttc_msg.append([msg_data[i].x, msg_data[i].y, msg_data[i].z])

        # print("len_ttc: {}".format(len(self.ttc_msg)))
        if len(self.ttc_msg):
            for i in range(len(self.ttc_msg)):
                cal_ttc = self.calculate_control(self.pos_i, self.ttc_msg[i])
                self.sum_v_ttc[0] += cal_ttc[0]
                self.sum_v_ttc[1] += cal_ttc[1]
                self.sum_v_ttc[2] += cal_ttc[2]
                self.ttc_dis += self.ttc_msg[i][2]
                # self.ttc_dis += (np.linalg.norm(np.array(
                #     [self.ttc_msg[i][0] - self.u0, self.ttc_msg[i][1] - self.v0, self.f], dtype=np.float64)))
        else:
            self.sum_v_ttc[0] = 0
            self.sum_v_ttc[1] = 0
            self.sum_v_ttc[2] = 0
        # self.ttc_msg.clear()

    def of_mssages_cb(self, msg):
        self.of_msg = []
        msg_data = msg.data
        cal_of = [0.0, 0.0, 0.0]
        self.sum_v_of = [0.0, 0.0, 0.0]
        self.of_dis = 0

        for i in range(len(msg_data)):
            self.of_msg.append([msg_data[i].x, msg_data[i].y, msg_data[i].z])

        # print("len_of: {}".format(len(self.of_msg)))
        if len(self.of_msg):
            for i in range(len(self.of_msg)):
                cal_of = self.calculate_control(self.pos_i, self.of_msg[i])
                self.sum_v_of[0] += cal_of[0]
                self.sum_v_of[1] += cal_of[1]
                self.sum_v_of[2] += cal_of[2]
                self.of_dis += self.of_msg[i][2]
                # self.of_dis += (np.linalg.norm(np.array(
                #     [self.of_msg[i][0] - self.u0, self.of_msg[i][1] - self.v0, self.f], dtype=np.float64)))
        else:
            self.sum_v_of[0] = 0
            self.sum_v_of[1] = 0
            self.sum_v_of[2] = 0
        # self.of_msg.clear()

    def par(self, d, d1, d2):
        if d < d1 or d == d1:
            return 1.0
        elif (d1 < d and d < d2) or d == d2:
            A = -2 / math.pow((d1 - d2), 3)
            B = 3 * (d1 + d2) / math.pow((d1 - d2), 3)
            C = -6 * d1 * d2 / math.pow((d1 - d2), 3)
            D = (math.pow(d2, 2) * (3 * d1 - d2)) / math.pow((d1 - d2), 3)
            return A * math.pow(d, 3) + B * math.pow(d, 2) + C * math.pow(d, 1) + D
        else:
            return 0.0

    def matrix(self, B, C):
        return np.array([[B[0] * C[0], B[0] * C[1], B[0] * C[2]],
                         [B[1] * C[0], B[1] * C[1], B[1] * C[2]],
                         [B[2] * C[0], B[2] * C[1], B[2] * C[2]]])

    def calculate_control(self, pos_i, ttc_msg):
        # calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = self.mav_R.dot(n_bc)
        n_co = np.array([ttc_msg[0] - self.u0, ttc_msg[1] -
                         self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = self.mav_R.dot(n_bo)

        # 0.707, 0.867 the parameter about smooth
        self.change = self.par(n_eo.dot(n_ec), 0.867, 0.966)
        # if pos_i[1] == 0 and ttc_msg[2] == 0:
        #     self.change = 1

        # calculate vod
        vd1 = self.matrix(n_eo, n_eo)
        matrix_I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        vI = matrix_I - vd1
        vod = (1/(1.001 - n_eo.dot(n_ec))) * \
            (1 - self.change) * self.kvod * vI.T.dot(n_ec)
        # print("vod:{}".format(vod))

        v_rd_b = np.array([1, 0, 0])
        v_rd_e = self.mav_R.dot(v_rd_b)
        # print("vr:{}".format(v_rd_e))

        v = ttc_msg[2]*(vod + self.change * v_rd_e)
        # v = self.sat(v,10)
        v[0] = self.sat(4*v[0], 0.2)  # 0.2
        v[1] = self.sat(4*v[1], 0.2)  # 0.2
        v[2] = self.sat(v[2], 0.1)  # 0.1
        # print("v:{}".format(v))
        return [v[0], v[1], v[2]]
        # print("Enter obs control!!!")
        # return [1, 1, 1]

    def obs_control(self):
        sum_v = [0.0, 0.0, 0.0]
        self.all_dis = 0
        # self.sum_v_ttc\ self.sum_v_of
        # ttc and of
        sum_v[0] = self.sum_v_ttc[0] + self.sum_v_of[0]
        sum_v[1] = self.sum_v_ttc[1] + self.sum_v_of[1]
        sum_v[2] = self.sum_v_ttc[2] + self.sum_v_of[2]

        # only ttc
        # sum_v[0] = self.sum_v_ttc[0]
        # sum_v[1] = self.sum_v_ttc[1]
        # sum_v[2] = self.sum_v_ttc[2]

        # only of
        # sum_v[0] = self.sum_v_of[0]
        # sum_v[1] = self.sum_v_of[1]
        # sum_v[2] = self.sum_v_of[2]

        # sum_v = self.SatIntegral()sat
        # sum_v = self.SatIntegral(sum_v, 0.3, -0.3)
        self.all_dis = self.ttc_dis + self.of_dis
        sum_v[1] = 0
        sum_v = self.sat(sum_v, 0.5)  # 0.3
        return [sum_v[0], sum_v[1], sum_v[2], 0]

    def mav_vel_cb(self, msg):
        self.mav_vel = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    # mav_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

    def pos_vel(self):
        # self.cmd_pos_vel = self.sat(
        #     self.P_pos * (self.desire_pos - self.mav_pos), 0.5)
        self.cmd_pos_vel = self.P_pos * (self.desire_pos - self.mav_pos)
        self.cmd_pos_vel[0] = self.sat(self.cmd_pos_vel[0], 0.5)  # 0.6   0.5
        self.cmd_pos_vel[1] = self.sat(self.cmd_pos_vel[1], 0.35)
        # pos_error = self.desire_pos - self.mav_pos
        # 保持进入路径跟随模式时，高度不发生变化
        self.cmd_pos_vel[2] = self.Saturation(
            self.P_heigh * (self.start_height - self.mav_pos[2]), 1, -1)
        # if self.distance > 8:
        # print("cmd_pos_vel: {}".format(self.cmd_pos_vel))

    # 保持朝目标方向
    def control_yaw(self):
        desire_yaw = math.atan2(
            self.desire_pos[1] - self.mav_pos[1], self.desire_pos[0] - self.mav_pos[0])
        # desire_yaw = -2.7  #1.2  -2.7
        desire_yaw = self.start_yaw
        dlt_yaw = self.minAngleDiff(desire_yaw, self.mav_yaw)
        self.cmd_yaw = self.Saturation(self.P_yaw * dlt_yaw, 0.2, -0.2)
        # pos_error = self.desire_pos - self.mav_pos
        # self.cmd_pos_vel[2] = 0
        print("desire_yaw: {}".format(desire_yaw))
        print("mav_yaw: {}".format(self.mav_yaw))
        print("cmd_yaw: {}".format(self.cmd_yaw))

    def sphere_control(self):
        obj_msg = Obj()
        # sphere_msg = Obj()
        nums = 25
        while nums > 0:
            obj_msg.id = ID_Tong[nums - 1]
            obj_msg.type = 99
            obj_msg.position.x = Pos_Tong[nums - 1][0]
            # print("obj_msg.position.x: {}".format(obj_msg.position.x))
            obj_msg.position.y = Pos_Tong[nums - 1][1]
            # print("obj_msg.position.y: {}".format(obj_msg.position.y))
            obj_msg.position.z = Pos_Tong[nums - 1][2]
            # print("obj_msg.position.z: {}".format(obj_msg.position.z))
            obj_msg.size.x = 1  # 0.4
            obj_msg.size.y = 1  # 0.4
            obj_msg.size.z = 0.8
            self.sphere_pub.publish(obj_msg)
            nums = nums - 1
        # sphere_nums = 8
        # while sphere_nums > 0:
        #     sphere_msg.id = ID_Sphere[sphere_nums - 1]
        #     sphere_msg.type = 152
        #     sphere_msg.position.x = Pos_Sphere[sphere_nums - 1][0]
        #     # print("obj_msg.position.x: {}".format(obj_msg.position.x))
        #     sphere_msg.position.y = Pos_Sphere[sphere_nums - 1][1]
        #     # print("obj_msg.position.y: {}".format(obj_msg.position.y))
        #     sphere_msg.position.z = Pos_Sphere[sphere_nums - 1][2]
        #     # print("obj_msg.position.z: {}".format(obj_msg.position.z))
        #     sphere_msg.size.x = 0.2
        #     sphere_msg.size.y = 0.2
        #     sphere_msg.size.z = 0.2
        #     self.sphere_pub.publish(sphere_msg)
        #     sphere_nums = sphere_nums - 1

    def minAngleDiff(self, a, b):
        diff = a - b
        if diff < 0:
            diff += 2*np.pi
        if diff < np.pi:
            return diff
        else:
            return diff - 2*np.pi

    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a/n*maxv
        else:
            return a

    def SatIntegral(self, a, up, down):
        for i in range(len(a)):
            if a[i] > up:
                a[i] = up
            elif a[i] < down:
                a[i] = down
        return a

    def Saturation(self, a, up, down):
        if a > up:
            a = up
        elif a < down:
            a = down
        return a

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg
        self.imu_rate = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    def gps_callback(self, msg):
        self.gps = msg

    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False


if __name__ == '__main__':
    con = Px4Controller()
    # con.read_kbd_input()
    # threading.Thread(target=con.read_kbd_input, args=())
    inputThread = threading.Thread(target=con.read_kbd_input)
    inputThread.start()
    con.start()
