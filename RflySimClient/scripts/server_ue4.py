#!/usr/bin/env python3
#coding=utf-8

# import required libraries
# pip3 install pymavlink pyserial

import cv2
import numpy as np
import time
import VisionCaptureApi
import PX4MavCtrlV4 as PX4MavCtrl
import math
import Perception
import time
import threading

# 启用ROS发布模式
VisionCaptureApi.isEnableRosTrans = True
Perception.isEnableRosTrans = True
vis = VisionCaptureApi.VisionCaptureApi()

# VisionCaptureApi 中的配置函数
vis.jsonLoad(1)  # 加载Config.json中的传感器配置文件
vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')

vis.RemotSendIP = "192.168.3.56"
# 控制飞机起飞到一定高度
VehilceNum = 1
# Create MAV instance
mav = PX4MavCtrl.PX4MavCtrler(20100, '192.168.3.4')
#mav = PX4MavCtrl.PX4MavCtrler()
time.sleep(2)
mav.sendUE4PosScale(2, 100, 0, [0, 0, -100], [0, 0, 0], [2, 2, 2])
mav.sendUE4PosScale(106, 3, 0, [2320, 100, -10], [0, 0, 0], [10, 10, 10])
mav.sendUE4PosScale(1, 200, 0, [0,0,0])
time.sleep(0.1)
mav.sendUE4PosScale(177, 400, 0, [7000,1000,0], [0,0,0], [10, 10, 10])  # 坦克，NED
time.sleep(2)
perception = Perception.Perception(mav, vis)
perception.AddDrones([1, 2])
perception.AddObj([177, 106])
#perception.AddObj(77)
pair = (0, 1)
perception.PTZCameraPair(pair)
perception.ReqRfySim()
time.sleep(3)
perception.StartCaptureImg()


def Fly(per: Perception.Perception):
    # 控制示例飞机飞行,这里只是示例，实际控制飞机应当由simulink控制
    while True:
        for idx, val in per.drones.items():
            if(val.copter_id == 1):
                val.position[0] = val.position[0] + 0.4  # 观测飞机4m/s
            else:
                val.position[0] = val.position[0] + 0.6  # 目标飞机6m/s
            per.mav.sendUE4PosScale(
                val.copter_id, 100, 0, val.position, [0, 0, 0], [2, 2, 2])
        time.sleep(0.1)


fly_th = threading.Thread(target=Fly, args=(perception,))
fly_th.start()

# Start MAV loop with UDP mode: MAVLINK_FULL

# Get the takeoff position of each vehicle to the UE4 Map
# this can be adopted to obtain the global position of a vehicle in swarm simulation

# lastTime = time.time()

# while True:
#     lastTime = lastTime + 1/30.0
#     sleepTime = lastTime - time.time()
#     if sleepTime > 0:
#         time.sleep(sleepTime)
#     else:
#         lastTime = time.time()
#     img = perception.GetImg(1,0)
#     if (len(img) ==0):
#         continue
#     cv2.imshow("test",img)
#     cv2.waitKey(1)

