#!/usr/bin/env python
#coding=utf-8

import numpy as np


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
        self.R_cb = np.array([[1,0,0],\
                             [0,0,1],\
                             [0,-1,0]])
        self.n_cc = np.array([0,0,1])


    def RotateAttackController(self, pos_info, pos_i, image_center):
        #calacute nc,the first idex(c:camera,b:body,e:earth) represent the frmae, the second idex(c,o) represent the camera or obstacle
        n_bc = self.R_cb.dot(self.n_cc)
        n_ec = pos_info["mav_R"].dot(n_bc)
        
        #calacute the no
        n_co = np.array([pos_i[0] - self.u0, pos_i[1] - self.v0, self.f], dtype=np.float64)
        n_co /= np.linalg.norm(n_co)
        n_bo = self.R_cb.dot(n_co)
        n_eo = pos_info["mav_R"].dot(n_bo)

        cos_beta = n_bo.dot(n_bc)
        v_b = n_bo*cos_beta - n_bc
        
        self.cnt += 1
        # v_m[1] = v_b[1] * 0.1/(1.01-cos_beta) + self.sat(self.cnt * 0.1,10)
        v_m = np.array([0., 0., 0.])
        # case1: (0.02, 3, 10)
        # case2: (0.04, 3, 12)
        # v_m[1] = self.sat(self.cnt * self.acc_rate_ibvs, self.max_v_ibvs-2)
        v_m[1] = self.max_v_ibvs-5
        v_m[0] = self.k_ibvs_hor*v_b[0]
        v_m[2] = self.sat(self.k_ibvs_ver*v_b[2], 5)
        # v_f = self.sat(self.cnt*0.02*np.array([0.,1.,0.]), 10)
        # v_m = (1-cos_beta)*v_b + (cos_beta)*v_f
        v = pos_info["mav_R"].dot(v_m)
        v = self.sat(v, self.max_v_ibvs)
        yaw_rate = self.k_ibvs_yaw*(image_center[0] - pos_i[0])
        
        print("v_b: {}\nv_m: {}\nv: {}".format(v_b, v_m, v))
        print("yaw_rate: {}".format(yaw_rate))
        return [v[0], v[1], v[2], yaw_rate]


    def sat(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a