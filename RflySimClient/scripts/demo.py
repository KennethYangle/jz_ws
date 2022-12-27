# 这个设置一个飞机上一个相机，前方10架飞机的

# import pygame as pg  # 调试使用，用来控制观测飞机运动，通过按方向键控制飞机飞行
import cv2
import numpy as np
import time
import VisionCaptureApi
import PX4MavCtrlV4 as PX4MavCtrl
import math
import Perception

import threading

# 启用ROS发布模式
VisionCaptureApi.isEnableRosTrans = True
Perception.isEnableRosTrans = True
vis = VisionCaptureApi.VisionCaptureApi()

# VisionCaptureApi 中的配置函数
vis.jsonLoad(1)  # 加载Config.json中的传感器配置文件
vis.startImgCap()  # 开启取图循环，执行本语句之后，已经可以通过vis.Img[i]读取到图片了
print('Start Image Reciver')

vis.RemotSendIP = "192.168.31.56"
# 控制飞机起飞到一定高度
VehilceNum = 1
# Create MAV instance
mav = PX4MavCtrl.PX4MavCtrler(20100, '255.255.255.255')

# 这架飞机上面通过配置文件，配置了一个可见光相机
mav.sendUE4PosScale(1, 100, 0, [0, 0, -60], [0, 0, 0], [1, 1, 1])

# 创建10加固定翼飞机,每架飞机上都有一个下视的可见光, 可以使用更简洁的方式去创建飞机，这里只是为了直观方便
# mav.sendUE4PosScale(10, 100, 0, [0, -30, -100], [0, 0, 0], [2, 2, 2])  # 左侧
# mav.sendUE4PosScale(11, 100, 0, [0, 30, -100], [0, 0, 0], [2, 2, 2])  # 右侧
# mav.sendUE4PosScale(12, 100, 0, [30, 0, -70], [0, 0, 0], [2, 2, 2])   # 前下
# mav.sendUE4PosScale(13, 100, 0, [-30, 0, -70], [0, 0, 0], [2, 2, 2])  # 后下
# mav.sendUE4PosScale(14, 100, 0, [30, 0, -130], [0, 0, 0], [2, 2, 2])  # 前上
# mav.sendUE4PosScale(15, 100, 0, [-30, 0, -130], [0, 0, 0], [2, 2, 2])  # 后上
# mav.sendUE4PosScale(16, 100, 0, [80, -25, -50], [0, 0, 0], [2, 2, 2])
# mav.sendUE4PosScale(17, 100, 0, [80, -40, -50], [0, 0, 0], [2, 2, 2])
# mav.sendUE4PosScale(18, 100, 0, [80, 10, -50], [0, 0, 0], [2, 2, 2])
# mav.sendUE4PosScale(19, 100, 0, [80, 30, -50], [0, 0, 0], [2, 2, 2])

# 地面有十家四旋翼无人机
mav.sendUE4PosScale(100, 3, 0, [60, 0, -10], [0, 0, 0], [2, 2, 2])
mav.sendUE4PosScale(101, 3, 0, [60, -20, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(102, 3, 0, [60, -30, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(103, 3, 0, [60, 20, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(104, 3, 0, [60, 40, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(105, 3, 0, [50, -5, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(106, 3, 0, [50, -20, -10], [0, 0, 0], [5, 5, 5])
mav.sendUE4PosScale(107, 3, 0, [50, -35, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(108, 3, 0, [50, 15, -10], [0, 0, 0], [1, 1, 1])
mav.sendUE4PosScale(109, 3, 0, [50, 25, -10], [0, 0, 0], [1, 1, 1])


# while True:
#     if vis.hasData[0]:
#         cv2.imshow("rgb", vis.Img[0])
#         cv2.waitKey()
#         break
################################## 初始化目标 ################################
# 创建一个感知模块类对象
perception = Perception.Perception(mav, vis)
perception.AddDrones(1)
# 初始化目标
for i in range(100, 110):
    obj = Perception.Obj(id=i)  # id为对应的copter_id
    perception.AddObj(obj)

perception.ReqRfySim()
pair = (0, 1)
perception.PTZCameraPair(pair)
perception.StartCaptureImg()


############################# 与RFlySim建立连接 ######################

# 自定义扫描方式为Z形


def Screen(per: Perception.Perception, copter_id, cam_id):
    camera = perception.GetCamera(copter_id, cam_id, 'infrared')
    max_rotation = np.array(camera.max_rotation) * 180/math.pi
    min_rotation = np.array(camera.min_rotation) * 180/math.pi
    flag_pitch = 1  # 控制扫描方向
    flag_yaw = 1
    offset_pitch = 1  # 每次移动1度，这里是为了演示，实际中应该是半个视场角度
    offset_yaw = 1
    while True:
        if(cam_id == 1 and per.AimWeapon(copter_id, cam_id)):
            # 锁定目标后，该相机不做扫描,当目标消失在视野，或者相机已到运动到约束条件仍然没有锁定目标，认为锁定失败，此时目标在视场内，但不能锁定,会继续做扫描动作,然后又做锁定动作，循环往复直至能锁定目标或者目标消失在视野
            print("aim weapon")
            time.sleep(0.01)
            continue
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


# 为每个相机创建一个进程,当然在一个进程里去控制
#for i in range(0, 6):
    #th = threading.Thread(target=Screen, args=(perception, 1, i))
    #th.start()

time.sleep(5)


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

# 控制copter_id为1的第1号相机旋转[roll,pitch,yaw]角度，单位degree
# perception.CtrlCameraRat(1, 1, [1, -40, 6])
# perception.CtrlCameraRat(1, 1, [1, -40, -6])

# 获取目标(可以是飞机或其他)位姿
# position, orietation = perception.GetTargetPose(
# 10)  # 获取copter_id为10的目标位姿，这里是飞机

# 可直接通过camera 获取相机的视场角，成像的数据宽与高，以及其他信息,但是要修改属性，通过函数修改
# 下视相机的位姿相对于全局坐标系的，其他的是相对载体坐标系的
# camera_rgb = perception.GetCamera(1, 1, 'rgb')  # 获取copter_id为10的第号可见光相机
# print("pos:", list(camera_rgb.position), ", FOV: ",
#       camera_rgb.FOV, ",ori", list(camera_rgb.orietation))

# perception.SetCameraFOV(1, 1, 90, 'rgb')  # 设置可见光视场角

# perception.SetCameraFOV(1, 1, 90, 'infrared')  # 设置可见光视场角

# camera = perception.GetCamera(1, 1, 'infrared')  # 获取copter_id为10的第1号红外相机

# camera = perception.GetCamera(1, 1, 'infrared')  # 获取copter_id为10的第1号红外相机
# print("pos:", camera.position, ", FOV: ",
#       camera.FOV, ",ori", camera.orietation)


# 修改copter_id为10,1号红外相机的视场角为60度(默认，1既是红外相机也是可见光相机的编号)
# perception.SetCameraFOV(1, 1, 30, 'rgb')  # 设置可见光视场角
# perception.SetCameraFOV(1, 1, 30, 'infrared')  # 设置红外相机视场角

# 通过这个函数修改相机属性(视场角，旋转角，位置)，ori=[roll,pitch,yaw]单位degree
# perception.SetCameraProperty(copter_id=10, cam_id=1, FOV=90, ori=[
#                              30, 40, 10], class_type='infrared')
