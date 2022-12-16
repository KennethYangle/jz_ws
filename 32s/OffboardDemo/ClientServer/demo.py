
# 构建一个盘旋式上升的飞机，通过旋转云台上的激光检测目标，设云台两个旋转自由度，约束条件[-60,60],[-90,90]，
# 需求是超天上扫描，那么约定传感器x方向为载体z的反方向,可认为载体为地面上固定的目标

import PX4MavCtrlV4 as PX4MavCtrl

import open3d as o3d

import numpy as np
import math
import time
import sys


import VisionCaptureApi

theta = np.linspace(0, 2 * math.pi, 360)
c_t = np.cos(theta)
c_t_t = np.cos(theta.T)
s_t = np.sin(theta)

center = [5, 5, -8]

extern_d = 8
cic_width = 3


def Run(mav: PX4MavCtrl, ext, cic_w):
    while True:
        for i in len(c_t):

            x = (ext + cic_w * c_t_t[i]) * c_t[i]
            y = (ext + cic_w*c_t_t[i]) * s_t[i]
            z = 0.1
        time.sleep(0.01)


mav = PX4MavCtrl.PX4MavCtrler()
mav.sendUE4PosScale(1, 3, 0, [0, 0, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(2, 3, 0, [0, 0, -10], [0, 0, 0], [1.333, 1.333, 1.333])

vis = VisionCaptureApi.VisionCaptureApi()
vis.jsonLoad()
isSuss = vis.sendReqToUE4()  # 向RflySim3D发送取图请求，并验证
if not isSuss:  # 如果请求取图失败，则退出
    sys.exit(0)
vis.startImgCap()
