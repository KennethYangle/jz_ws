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
import rospy
from rflysim_msgs.msg import CameraInfo
from rospy import timer

# 启用ROS发布模式
VisionCaptureApi.isEnableRosTrans = True
Perception.isEnableRosTrans = True
vis = VisionCaptureApi.VisionCaptureApi()


# VisionCaptureApi 中的配置函数
vis.jsonLoad(1)  # 加载Config.json中的传感器配置文件
vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')



#创建ros 发布器
pub_list = []
camera_info = []
for i in range(len(vis.VisSensor)):
    if(vis.VisSensor[i].TypeID == 1 or vis.VisSensor[i].TypeID ==7 ):
        # camera_info += [CameraInfo]
        pub_list += [rospy.Publisher("/rflysim/camera"+ str(i) + "_info",CameraInfo,queue_size=30)]

        



RflySimIP = '192.168.3.22'
scale_width = 0.2 #占据图像宽度的比例
def AutoAdaptCamera(per: Perception.Perception, copter_id, cam_id,scale_w):
    '''
    per: 红外接受主要类；
    copter_id: 需要控制的相机在哪架飞机上
    cam_id: 需要控制的相机编号
    '''
    
    camera = perception.GetCamera(copter_id, cam_id)
    max_rotation = np.array(camera.max_rotation) * 180/math.pi
    min_rotation = np.array(camera.min_rotation) * 180/math.pi
    flag_pitch = 1  # 控制扫描方向
    flag_yaw = 1
    offset_pitch = 20  # Z字形扫描 每次pitch方向移动度数，这里是为了演示，实际中根据需求更改
    offset_yaw = 1  # Z字形每次扫描yaw方向移动度数
    is_weapon = False
    while True:
        # print("aim test")
        if(per.AimWeapon(camera, 0,RflySimIP,scale_w) and not is_weapon):
            # 锁定目标后，该相机不做扫描,当目标消失在视野，或者相机已到运动到约束条件仍然没有锁定目标，认为锁定失败，此时目标在视场内，但不能锁定,会继续做扫描动作,然后又做锁定动作，循环往复直至能锁定目标或者目标消失在视野
            # print("aim weapon")
            # time.sleep(0.01)
            is_weapon = True
        # 没有目标的时候接着扫描
        # if (is_weapon or per.AimWeapon(camera, 1,RflySimIP,scale_w)):
        if (is_weapon ):
            # print("adapt camera fov")
            per.AimWeapon(camera, 1,RflySimIP,scale_w)
            continue
        is_weapon = False  # 重新扫描
        ori = per.GetCameraOri(copter_id, cam_id)
        ctrl_yaw = flag_yaw * offset_yaw + ori[2]
        ctrl_pitch = ori[1]
        if(ctrl_yaw < min_rotation[2] or ctrl_yaw > max_rotation[2]):
            flag_yaw = -flag_yaw
            if(ctrl_yaw < min_rotation[2]):
                ctrl_yaw = min_rotation[2]
            if(ctrl_yaw > max_rotation[2]):
                ctrl_yaw = max_rotation[2]
            ctrl_pitch = ctrl_pitch + offset_pitch * flag_pitch
        if(ctrl_pitch < min_rotation[1] or ctrl_pitch > max_rotation[1]):
            flag_pitch = -flag_pitch
            if(ctrl_pitch < min_rotation[1]):
                ctrl_pitch = min_rotation[1]
            if(ctrl_pitch > max_rotation[1]):
                ctrl_pitch = max_rotation[1]
        if(cam_id == 1):
            print("camera_id: ", cam_id, "pitch: ",
                  ctrl_pitch, " yaw: ", ctrl_yaw)
        per.CtrlCameraRat(copter_id, cam_id, [ori[0], ctrl_pitch, ctrl_yaw])
        time.sleep(0.1)

vis.RemotSendIP = "192.168.3.56"
# 控制飞机起飞到一定高度
VehilceNum = 1
# Create MAV instance
mav = PX4MavCtrl.PX4MavCtrler(20100, RflySimIP)
#mav = PX4MavCtrl.PX4MavCtrler()
time.sleep(2)
mav.sendUE4PosScale(2, 100, 0, [0, 0, -100], [0, 0, 0], [2, 2, 2])
# mav.sendUE4PosNew(106,3,[0,0,0],[0,0,0],[0,0,0],[83.535,38.721,1134.264,0,0,0,0,0], 0)
mav.sendUE4PosScale(106, 3, 0, [2320, 100, -10], [0, 0, 0], [10, 10, 10])
# mav.sendUE4PosNew(106,3,[0,0,0],[0,0,0],[0,0,0],[83.562,38.720,6145.686,0,0,0,0,0], 0)
mav.sendUE4PosScale(1, 200, 0, [0,0,0])
time.sleep(0.1)
# mav.sendUE4PosScale(177, 400, 0, [7000,1000,-10], [0,0,0], [30, 30, 30])  # 坦克，NED
# mav.sendUE4PosScale(177, 400, 0, [22018, 17913, -178.5], [0,0,0], [30, 30, 30])  # 坦克，NED
mav.sendUE4PosScale(177, 601, 0, [22018, -17913, -178.5], [0,0,0], [30, 30, 30])  # 船，NED
# mav.sendUE4PosNew(177,400,[0,0,0],[0,0,0],[0,0,0],[83.616,38.712,1149.179,0,0,0,0,0], 0)
time.sleep(1)
mav.sendUE4ExtAct(177, [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
time.sleep(2)
perception = Perception.Perception(mav, vis)
perception.AddDrones([1])
perception.AddObj([177])
#perception.AddObj(77)
pair = (0, 1)
perception.PTZCameraPair(pair)
perception.ReqRfySim()
time.sleep(3)

perception.StartCaptureImg()

th = threading.Thread(target=AutoAdaptCamera, args=(perception, 1, 0,scale_width))
th.start()




def PublishCamera(event):
    global vis
    global pub_list
    global camera_info
    global perception
    copter_id = 1
    drone = perception.drones[str(copter_id)]
    for i in range(len(vis.VisSensor)):
        if(vis.VisSensor[i].TypeID == 1 or vis.VisSensor[i].TypeID ==7 ):
            cam = perception.GetCamera(1,vis.VisSensor[i].SeqID)
            msg_cam = CameraInfo()
            msg_cam.seq_id = int(vis.VisSensor[i].SeqID)
            msg_cam.data_height = int(cam.data_height)
            msg_cam.data_width = int(cam.data_width)
            msg_cam.FOV = int(cam.GetFOV())
            #因为这里都是相对与全局坐标系的
            #ori_world_to_v = np.array(cam.GetOrientation()) - np.array(drone.orientation)
            c2v_matrix = cam.trans_R * drone.trans_R.T #3 x 3
            #rpy顺序
            beta = -np.arcsin(c2v_matrix[2,0])
            alpha = np.arctan2(c2v_matrix[2,1]/np.cos(beta),c2v_matrix[2,2]/np.cos(beta))
            gamma = np.arctan2(c2v_matrix[1,0]/np.cos(beta),c2v_matrix[0,0]/np.cos(beta))
            
            msg_cam.roll = beta
            msg_cam.pitch = alpha
            msg_cam.yaw = gamma
            matrix = cam.internal_matrix
            msg_cam.K[0] = matrix[0,0]
            msg_cam.K[2] = matrix[0,2]
            msg_cam.K[4] = matrix[1,1]
            msg_cam.K[5] = matrix[1,2]
            msg_cam.K[8] = 1
            msg_cam.R = c2v_matrix.flatten().tolist()[0]
            # print("cam.R:", msg_cam.R)
            pub_list[i].publish(msg_cam)
        



rospy.Timer(rospy.Duration(1/30),PublishCamera)

# lastTime = time.time()
# while True:
#     sleepTime = lastTime - time.time()
#     if sleepTime > 0:
#         time.sleep(sleepTime)
#     else:
#         lastTime = time.time() + 1/30.0
#         # Process your image here
#         print("show img")
#         img = perception.GetImg(1, 0)
#         if(len(img) == 0):
#             continue
#         cv2.imshow('Img', img)
#         cv2.waitKey(1)



# 为每个相机创建一个进程,当然在一个进程里去控制
# for i in range(0, 6):



# time.sleep(5)


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

