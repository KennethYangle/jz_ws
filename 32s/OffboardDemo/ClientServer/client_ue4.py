# -*- coding: utf-8 -* -
import cv2
import sys
import time
import VisionCaptureApi
import os
import PX4MavCtrlV4 as PX4MavCtrl
import Perception
# The IP should be specified by the other computer
# mav = PX4MavCtrl.PX4MavCtrler(20100, "255.255.255.255")


# mav.sendUE4PosScale(1, 100, 0, [0, 0, -100], [0, 0, 0], [2, 2, 2])
# mav.sendUE4PosScale(106, 3, 0, [50, -20, -10], [0, 0, 0], [5, 5, 5])
# 创建10加固定翼飞机,每架飞机上都有一个下视的可见光, 可以使用更简洁的方式去创建飞机，这里只是为了直观方便

vis = VisionCaptureApi.VisionCaptureApi()
# Send command to UE4 Window 1 to change resolution
# 设置UE4窗口分辨率，注意本窗口仅限于显示，取图分辨率在json中配置，本窗口设置越小，资源需求越少。
vis.sendUE4Cmd(b'r.setres 720x405w', 0)
vis.sendUE4Cmd(b't.MaxFPS 30', 0)  # 设置UE4最大刷新频率，同时也是取图频率
time.sleep(2)

# VisionCaptureApi 中的配置函数
vis.jsonLoad(1)  # 加载Config.json中的传感器配置文件
isSuss = vis.sendReqToUE4()  # 向RflySim3D发送取图请求，并验证
if not isSuss:  # 如果请求取图失败，则退出
    sys.exit(0)
# vis.RemotSendIP = '192.168.31.24'  # 如果是分布式布局,得改配置文件里面的IP
# 注意，手动修改RemotSendIP的值，可以将图片发送到远端Linux电脑的IP地址
# 如果不修改这个值，那么发送的IP地址为json文件中SendProtocol[1:4]定义的IP
# 图片的发送端口，为json中SendProtocol[5]定义好的。

vis.isUE4DirectUDP=True
# 注意，手动修改本命令能强制将图片发送机制为UE4直接发出UDP图片到指定IP地址
# 如果不修改这个值，那么发送机制由json文件中SendProtocol[0]中定义


# percepoint = Perception.Perception(mav, vis)
# percepoint.AddDrones(1)  # 可以是列表，也可以是int数值
# percepoint.AddObj(106)  # 可以是列表，也可以是int数值
# percepoint.ReqRfySim()

# isSuss = vis.sendImuReqCopterSim(
# copterID=1, IP='192.168.31.88', port=31000, freq=200)
# isSuss = vis.sendImuReqClient()
# if not isSuss:
#     sys.exit(0)

# 注意：这里不需要调用startImgCap()来去图，本程序只是发送数据请求

print('Image UDP direct command has sent to RflySim3D')
