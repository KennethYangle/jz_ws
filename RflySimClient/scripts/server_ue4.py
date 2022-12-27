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
mav = PX4MavCtrl.PX4MavCtrler(20100, '255.255.255.255')
#mav = PX4MavCtrl.PX4MavCtrler()
time.sleep(2)
mav.sendUE4PosScale(2, 100, 0, [0, 0, -100], [0, 0, 0], [2, 2, 2])
mav.sendUE4PosScale(106, 3, 0, [50, -20, -10], [0, 0, 0], [5, 5, 5])
mav.sendUE4PosScale(77, 400, 0, [75,0,-1.7], [0,0,0], [1, 1, 1])
perception = Perception.Perception(mav, vis)
perception.AddDrones(2)
perception.AddObj(106)
perception.AddObj(77)
perception.ReqRfySim()
pair = (0, 1)
perception.PTZCameraPair(pair)
perception.StartCaptureImg()


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

#     for i in range(len(vis.hasData)):
#         if vis.hasData[i]:
#             # Process your image here
#             cv2.imshow('Img'+str(i), vis.Img[i])
#             cv2.waitKey(1)

