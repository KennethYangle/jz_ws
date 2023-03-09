#!/usr/bin/env python
#coding=utf-8

import numpy as np
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_matrix, euler_from_matrix, quaternion_from_euler

class Utils(object):

    def __init__(self, params):
        self.WIDTH = params["WIDTH"] #simulation 720  Real_flight:640
        self.HEIGHT = params["HEIGHT"] #simulation 405  Real_flight:405
        self.k_ibvs_hor = params["k_ibvs_hor"]
        self.k_ibvs_ver = params["k_ibvs_ver"]
        self.k_ibvs_yaw = params["k_ibvs_yaw"]
        self.acc_rate_ibvs = params["acc_rate_ibvs"]
        self.max_v_ibvs = params["max_v_ibvs"]

        self.w, self.h = self.WIDTH, self.HEIGHT
        self.u0 = self.w/2
        self.v0 = self.h/2
        #realsense: fx:632.9640658678117  fy:638.2668942402212
        self.f = params["F"] #346.6  # 这个需要依据实际情况进行设定flength=(width/2)/tan(hfov/2),不同仿真环境以及真机实验中需要依据实际情况进行修改

        self.cnt = 1
        #camrea frame to mavros_body frame
        self.R_c0b = np.array([[1,0,0],\
                             [0,0,1],\
                             [0,-1,0]])
        self.n_cc = np.array([0,0,1])
        #FRD frame to FLU frame
        self.R_b_trans = np.array([[1,0,0],\
                                   [0,-1,0],\
                                   [0,0,-1]])


    def RotateAttackController(self, pos_info, pos_i, image_center, cam_info):
        #calculate nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_c0b.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calculate the no
        # n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, cam_info["foc"]], dtype=np.float64)
        # n_co /= np.linalg.norm(n_co)
        # n_bo = self.R_c0b.dot(n_co)
        # n_eo = pos_info["mav_R"].dot(n_bo)
        n_co = np.array([cam_info["foc"], pos_i[0] - self.u0, pos_i[1] - self.v0], dtype=np.float64)    # 相机系也定义为FRD
        n_co /= np.linalg.norm(n_co)
        n_bo = cam_info["R"].dot(n_co)      # 转到载具系FRD
        n_eo = pos_info["mav_R"].dot(self.R_b_trans.dot(n_bo))  # 先转到FLU再转到ENU
        
        # calculate the v_d and a_d
        g = np.array([0, 0, 9.8])
        V = np.linalg.norm(pos_info["mav_vel"])
        v_d = (V + 1) * n_eo
        a_d = 0.6 * (v_d - pos_info["mav_vel"])

        # calculate R_d
        r1 = pos_info["mav_vel"] / V
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
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/7.9, euler[2])     # ly初值打击，target_pos = np.array([1791.3, 5201.8, 17.85])，takeoff_pos=[1305, 4933, 97]
        q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/9, euler[2])        # ly初值打击+盘旋，target_pos = np.array([1791.3, 2201.8, 17.85])，takeoff_pos=[1305, 1733, 97]
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/17.5, euler[2])
        # q_array = quaternion_from_euler(-euler[0], -euler[1]+np.pi/30, euler[2])

        q = Quaternion()
        # q.w = 0.996
        # q.x = 0.087
        q.x, q.y, q.z, q.w = q_array[0], q_array[1], q_array[2], q_array[3]
        thrust = 0.75

        print("v_d: {}\nv: {}\n".format(v_d, pos_info["mav_vel"]))
        print("q: {}\nthrust: {}\n".format(q, thrust))
        return [q, thrust]


    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a