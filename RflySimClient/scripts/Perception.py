'''

根据需求,红外相机用Python虚构的,可见光用RFlySim输出的
说明: 因为红外相机是虚构的,运动通过多次运动扫描合成的红外成像,在虚构相机里直接更改相机视场角即可

'''
import copy
import math
import sys
import threading
import time

import cv2
import numpy as np

import PX4MavCtrlV4 as PX4MavCtrl  # 用来控制飞机以及获取目标信息
import VisionCaptureApi  # 相机取图
import platform

isLinux = False
if platform.system().lower() == 'linux':
    isLinux = True
    try:
        from logging import exception
        from typing import Any
        from xml.etree.ElementTree import tostring
        import yaml
        import rospy
        import sensor_msgs.msg as sensor
        import std_msgs.msg as std_msg

    except ImportError:
        print('Faild to load ROS labs')

# 是否启用ROS图像转发功能
# IsEnable ROS image topic forwarding
isEnableRosTrans = False


class Obj:
    def __init__(self, id: int, position=[0, 0, 0], orientation=[0, 0, 0], class_type='obj'):
        '''
        position: x,y,z坐标,所有坐标系,采取右手系(地,北,东)坐标系,所以初始化的是需要注意,
        orientation: rpy(roll,pitch,yaw 单位与平台保持一致,默认是degree),如果是四元数(x,y,z,w)
        '''
        self.RFlySimObj: any
        self.isHited = False  # 目标被撞击
        self.id = id
        self.position = position
        self.class_type = class_type
        # orientation 顺序分别是绕x，y,z轴旋转顺序，即(r(roll),p(pitch),y(yaw))
        self.orientation = [math.radians(orientation[0]), math.radians(
            orientation[1]), math.radians(orientation[2])]
        self.is_destroy = False  # 需要知道目标什么时候被集中销毁之类的，这个后续再确定
        self.bounding_box = []  # 目标bounding box 各方向长度的一般
        self.box_origin = []   # 目标bounding box 几何中心位置
        self.box_apex = []  # Bbox前八个元素为顶点，后面的点根据需求增加
        self.refresh_time = 0
        self.trans_R = []
        self.hasUpdate = False

    def ChangObjStatus(self, rflysim_obj, is_Hite: bool, id, pos: list, ori: list, is_destory: bool, bbox: list, ref_time, class_type: str):
        '''
            ori: 欧拉角,单位:角度

        '''
        self.RFlySimObj = rflysim_obj
        self.isHited = False  # 目标被撞击
        self.id = id
        self.position = pos
        self.class_type = class_type
        # orientation 顺序分别是绕x，y,z轴旋转顺序，即(r(roll),p(pitch),y(yaw))
        self.orientation = [math.radians(ori[0]), math.radians(
            ori[1]), math.radians(ori[2])]
        self.is_destroy = False  # 需要知道目标什么时候被集中销毁之类的，这个后续再确定
        self.bounding_box = bbox
        self.refresh_time = ref_time
        self.trans_R = np.matrix()

    def UpdateMatrix(self) -> bool:
        if(len(self.orientation) < 3 or len(self.position) < 3):
            return False
        # print("posUE ", self.position, "angEuler: ", self.orientation)
        rpy = self.orientation
        cos_r = math.cos(rpy[0])
        cos_p = math.cos(rpy[1])
        cos_y = math.cos(rpy[2])
        sin_r = math.sin(rpy[0])
        sin_p = math.sin(rpy[1])
        sin_y = math.sin(rpy[2])
        self.trans_R = np.matrix([[cos_p*cos_y, sin_r*sin_p*cos_y - cos_r*sin_y, cos_r*sin_p*cos_y + sin_r*sin_y],
                                  [cos_p*sin_y, sin_r*sin_p*sin_y + cos_r*cos_y,
                                   cos_r*sin_p*sin_y - sin_r*cos_y],
                                  [-sin_p, sin_r*cos_p, cos_r *
                                   cos_p]]
                                 )
        return True

    def CalBBoxApex(self):
        # 调用这个函数除了bbox,box_origin还有数据之外，同时默认bounding_box方向既是目标物体的方向(人为指定的方向)
        # 默认目标物体的坐标系也是使用(地，北，东)方向的坐标系
        if(len(self.bounding_box) == 0 or len(self.box_origin) == 0):
            print("Cal bounding box's aptex fial")
            sys.exit(0)
        self.box_apex = []
        self.box_apex.append(
            [self.bounding_box[0], -self.bounding_box[1], -self.bounding_box[2]])  # 前左上顶点
        self.box_apex.append(
            [self.bounding_box[0], self.bounding_box[1], -self.bounding_box[2]])  # 前右上顶点
        self.box_apex.append(
            [self.bounding_box[0], self.bounding_box[1], self.bounding_box[2]])  # 前右下顶点
        self.box_apex.append(
            [self.bounding_box[0], -self.bounding_box[1], self.bounding_box[2]])  # 前左下顶点
        self.box_apex.append(
            [-self.bounding_box[0], -self.bounding_box[1], -self.bounding_box[2]])  # 后左上顶点
        self.box_apex.append(
            [-self.bounding_box[0], self.bounding_box[1], -self.bounding_box[2]])  # 后右上顶点
        self.box_apex.append(
            [-self.bounding_box[0], self.bounding_box[1], self.bounding_box[2]])  # 后右下顶点
        self.box_apex.append(
            [-self.bounding_box[0], -self.bounding_box[1], self.bounding_box[2]])  # 后左下顶点
        box_apex = np.array(self.box_apex) + np.array(self.box_origin)
        self.box_apex = box_apex.tolist()


class Camera(Obj):
    # id：为相对于载体的编号
    def __init__(self, sensor: VisionCaptureApi.VisionSensorReq):
        '''
        '''
        super().__init__(id, class_type='camera')
        self.seq_id = sensor.SeqID
        self.FOV = sensor.CameraFOV
        # self.position = [0, 0, 0]  # 相对于载体位置,如果是RFlySim返回的数据是相对全局的
        # self.orientation = [0, 0, 0]  # 相对于载体姿态
        self.data_width = sensor.DataWidth
        self.data_height = sensor.DataHeight
        self.belong_copter = sensor.TargetCopter
        self.type = sensor.TypeID
        # self.data = []
        self.data_time = 0
        self.internal_matrix = []  # 相机内参矩阵,仿真没有畸变
        self.insight_obj = []  # 用来存取当前相机内的目标编号
        self.mean = 0  # 高斯噪声均值
        self.sigma = 50
        self.half_w = self.data_width/2
        self.half_h = self.data_height/2
        self.rate = sensor.DataCheckFreq
        self.joint_type = 'floating'
        if((np.abs(np.array(sensor.MaxRotaion) - np.array(sensor.MinRotaion)) < 1e-10).all()):
            self.joint_type = 'fixed'
        if(self.type == 7):
            self.orientation = (
                np.array(sensor.SensorAngEular) * math.pi/180).tolist()
        # 相机旋转的约束，下面分别是相机绕相机初始位置坐标系旋转,因此得算上云台安装在载体的角度
        self.min_rotation = ((np.array(
            sensor.MinRotaion) + np.array(sensor.SensorAngEular)) * math.pi / 180).tolist()
        self.max_rotation = ((np.array(
            sensor.MaxRotaion) + np.array(sensor.SensorAngEular)) * math.pi / 180).tolist()
        self.is_from_rflysim = False
        # if(type == 'rgb'):
        # self.is_from_rflysim = True

        if(len(self.internal_matrix) == 0 and self.FOV > 0 and self.half_h > 0 and self.half_w > 0):
            t_v = math.tan(math.radians(self.FOV/2))
            fx = self.half_w / t_v
            fy = self.half_w / t_v
            self.internal_matrix = np.matrix([[fx, 0, self.half_w, 0],
                                              [0, fy, self.half_h, 0],
                                              [0, 0, 0, 1]
                                              ])  # 3x4矩阵,为了方便计算

    def UpdateInternalMatrix(self):
        if(self.FOV > 0 and self.half_w > 0 and self.half_h):
            t_v = math.tan(math.radians(self.FOV/2.0))
            fx = self.half_w / t_v
            fy = self.half_w / t_v
            self.internal_matrix = np.matrix([[fx, 0, self.half_w, 0],
                                              [0, fy, self.half_h, 0],
                                              [0, 0, 0, 1]
                                              ])  # 3x4矩阵,为了方便计算
    # def SetCameraProperty(self, vis, positoin=[], oritation=[], FOV=0, img_w=0, img_h=0):
    #     if(self.seq_id)

    def SetFOV(self, val):
        self.FOV = val
        self.UpdateInternalMatrix()

    def SetPosition(self, val: list):
        self.position = copy.deepcopy(val)

    def SetOrientation(self, val: list) -> bool:  # 暂时不考虑四元数
        '''
        对相机的旋转，需要判断相机旋转的约束条件，如相机旋转自由度只有两个，那么另一个维度的约束条件不要设置0
        '''
        rad = np.array([math.radians(val[0]), math.radians(
            val[1]), math.radians(val[2])])
        if((rad < np.array(self.min_rotation)).all() and (rad > np.array(self.max_rotation)).all()):
            print("the val over constraint condition")
            return False
        self.orientation = rad.tolist()
        self.UpdateMatrix()

    def SetPose(self, position, orientation):
        self.SetPosition(position)
        self.SetOrientation(orientation)

    def SetImgSize(self, width, height):
        self.data_height = height
        self.data_width = width
        self.half_h = height/2
        self.half_w = width / 2
        self.UpdateInternalMatrix()

    def GetFOV(self):
        return self.FOV

    def GetPosition(self) -> list:
        return copy.deepcopy(self.position)

    def GetOrientation(self) -> list:
        return [math.degrees(self.orientation[0]), math.degrees(self.orientation[1]), math.degrees(self.orientation[2])]

    def GetDataSize(self) -> tuple:
        return(self.data_width, self.data_height)


class Drone(Obj):
    def __init__(self, position=[0, 0, 0], orientation=[0, 0, 0], copter_id=0):
        '''
        position: 载体坐标,需要注意的是载体坐标系原点与世界中载体坐标是否吻合,不然各传感器的转换全是错的
        orientation: 在世界坐标系下的姿态
        copter_id: 飞机的在RflySim中的唯一标志,控制飞机需要使用copter_id
        '''
        super().__init__(position=copy.deepcopy(position),
                         orientation=copy.deepcopy(orientation), id=copter_id, class_type='drone')
        self.RFlySimObj = any
        self.infrareds = {}  # 该飞机拥有的红外相机
        self.RGBCamera = {}
        self.is_destroy = False
        self.copter_id = copter_id
        self.tail_flame_len = 3  # 固定翼尾焰长度
        self.tail_flame_rad = 0.5  # 固定翼尾焰半径
        self.tail_flame_res = 10  # 在尾焰环上每10度取一点，角度越小，成像逼近圆弧，当然角度需要越吃计算需要和分辨率以及距离，综合考虑该值
        pass

    def AddInfrared(self, infrared: Camera):
        if(not str(infrared.seq_id) in self.infrareds):
            if(str(infrared.seq_id) in self.RGBCamera.keys()):
                infrared.is_from_rflysim = True
            self.infrareds[str(infrared.seq_id)] = infrared
        else:
            print("infrared sensor's id is conficted")

    def AddRGBCamera(self, cam: Camera):
        if not str(cam.id) in self.RGBCamera:
            cam.is_from_rflysim = True
            self.RGBCamera[str(cam.id)] = cam
            if(str(cam.id) in self.infrareds.keys()):
                self.infrareds[str(cam.id)].is_from_rflysim = True
        else:
            print("rgb sensor's id is conficted")

    def SetPose(self, position, orientation):
        # 需要更新相机的位置
        self.position = copy.deepcopy(position)
        self.orientation = copy.deepcopy(orientation)
        pass

    def SetCamRotationOffset(self, yaw, pitch, roll, camera_id):
        if(str(camera_id) in self.RGBCamera):
            ori = self.RGBCamera[str(camera_id)].orientation
            if(len(ori) == 3):
                ori[0] = yaw + ori[0]
                ori[1] = pitch + ori[1]
                ori[2] = roll + ori[2]
        else:
            print("not exist camera")

    def SetCamRotationEuler(self, yaw, pitch, roll, camera_id):
        if(not str(camera_id) in self.RGBCamera):
            print("not exist camera")
            return
        ori = self.RGBCamera[str(camera_id)].orientation
        if(len(ori) == 3):
            ori[0] = yaw
            ori[1] = pitch
            ori[2] = roll
        elif(len(ori) == 0):
            ori = ori + [yaw]
            ori = ori + [pitch]
            ori = ori + [roll]
        else:
            print("set camera rotation fail")


class Perception:
    def __init__(self, mav: PX4MavCtrl.PX4MavCtrler, vis: VisionCaptureApi.VisionCaptureApi):
        self.mav = mav
        self.vis = vis
        self.drones = {}
        self.objs = {}
        self.inter_target = {}
        self.is_init = False
        self.rgb_camera = {}
        self.inf_camera = {}
        self.pairs = {}

        # self.mav.sendUE4Cmd(b'RflyReqVehicleData 127.0.0.1')
        for sensor in vis.VisSensor:
            copter_id = sensor.TargetCopter
            if(not str(copter_id) in self.drones.keys()):
                self.drones[str(copter_id)] = Drone(copter_id=copter_id)
            cam = Camera(sensor)
            if str(copter_id) in self.drones.keys():
                drone = self.drones[str(copter_id)]
                if(sensor.TypeID == 7):
                    drone.AddInfrared(cam)
                    self.inf_camera[str(sensor.SeqID)] = copter_id
                elif(sensor.TypeID == 1):
                    drone.AddRGBCamera(cam)
                    self.rgb_camera[str(sensor.SeqID)] = copter_id

    def AddDrones(self, copter_ids):
        if isinstance(copter_ids, list):
            for id in copter_ids:
                if str(id) in self.drones.keys():
                    continue
                drone = Drone(copter_id=id)
                self.drones[str(id)] = drone
        elif isinstance(copter_ids, int):
            if not str(copter_ids) in self.drones.keys():
                self.drones[str(copter_ids)] = Drone(copter_id=copter_ids)
        else:
            print("the drone has exist")

    def PTZCameraPair(self, pairs):
        '''
        下视可见和红外共用一个云台
        piars:为元组列表，[(rgb seq_id1,infrared seq_id2),.....]
        '''
        if isinstance(pairs, list):
            for pair in pairs:
                if str(pair[0]) in self.rgb_camera.keys() and str(pair[1]) in self.inf_camera.keys():
                    if(self.rgb_camera[str(pair[0])] != self.inf_camera[str(pair[1])]):
                        print("the ", pair[0], " and ",
                              pair[1], " not belong same vehicle")
                        sys.exit(0)
                    else:
                        self.pairs[self.rgb_camera[str(pair[0])]] = pair
        elif isinstance(pairs, tuple):
            if str(pairs[0]) in self.rgb_camera.keys() and str(pairs[1]) in self.inf_camera.keys():
                if(self.rgb_camera[str(pairs[0])] != self.inf_camera[str(pairs[1])]):
                    print("the ", pairs[0], " and ", pairs[1],
                          " not belong same vehicle")
                    sys.exit(0)
                else:
                    self.pairs[self.rgb_camera[str(pairs[0])]] = pairs

    def AddObj(self, objs_id):
        if isinstance(objs_id, list):
            for id_ in objs_id:
                if not str(id_) in self.objs.keys():
                    self.objs[str(id_)] = Obj(id=id_)
        if isinstance(objs_id, int) and not str(objs_id) in self.objs.keys():
            self.objs[str(objs_id)] = Obj(id=objs_id)

    def AddInterTarget(self, target_id):
        if isinstance(target_id, list):
            for id_ in target_id:
                if not str(id_) in self.objs.keys():
                    self.inter_target[str(id_)] = Obj(id=id_)
        if isinstance(target_id, int) and not str(target_id) in self.inter_target.keys():
            self.inter_target[str(target_id)] = Obj(id=target_id)

    def AddObjAtRun(self):
        pass

    def AddDroneAtRun(self):
        pass

    def ReqRfySim(self):
        '''
        与RflySim 建立连接用
        '''
        if(len(self.drones) == 0):
            print("please add drone")
            sys.exit(0)

        objs = []
        cams = []
        for copter_id, val in self.drones.items():
            # self.mav.reqCamCoptObj(1, copter_id)
            objs = objs + [copter_id]
            for cam_id, cam in val.RGBCamera.items():
                cams = cams + [cam.seq_id]
                # self.mav.reqCamCoptObj(0, cam.seq_id)
        for copter_id, val in self.objs.items():
            # self.mav.reqCamCoptObj(1, copter_id)
            objs = objs + [copter_id]
        for copter_id, val in self.inter_target.items():
            objs = objs + [copter_id]
        self.mav.reqCamCoptObj(1, objs)
        self.mav.reqCamCoptObj(0, cams)

        time.sleep(2)

        self.mav.initUE4MsgRec()
        time.sleep(3)

        self.is_init = True

        # 创建接受线程
        # img_thread = threading.Thread(target=self.RecvCarmeraImg)
        # img_thread.start()

        # recv_thread = threading.Thread(target=self.RecvRflySimInfo)
        # recv_thread.start()

        # coll_thread = threading.Thread(target=self.CollisionStatus)
        # coll_thread.start()

        # self.RecvRflySimInfo()

    def RecvCarmeraImg(self):
        '''暂未使用'''
        cam_ = []
        for copter_id, val in self.drones.items():
            for cam_id, cam in self.drone[copter_id].RGBCamera.items():
                cam_ = cam_ + [cam]
        while True:
            for i in len(cam_):
                if self.vis.hasData[cam_[i].seq_id]:
                    self.vis.Img_lock[cam_[i].seq_id].acquire()
                    cam_[i].data = self.vis.Img[cam_[i].seq_id]
                    cam_[i].data_time = self.vis.timeStmp[cam_[i].seq_id]
                    self.vis.Img_lock[cam_[i].seq_id].release()
                    self.vis.hasData[cam_[i].seq_id] = False
            time.sleep(0.001)

    def RecvRflySimInfo(self):
        # to_do@daivesi,如果使用一个线程去接受数据，可能会延迟情况,后续再优化
        # 在这个线程执行之前，所有PrecePtion objs，drone 变量都以及设置完成
        print("total copter: ", len(self.mav.CoptDataVect))
        print("total camera: ", len(self.mav.CamDataVect))
        recv_ = []
        for copter_id, val in self.drones.items():
            drone = self.mav.getCamCoptObj(1, int(copter_id))
            if not isinstance(drone, PX4MavCtrl.CoptReqData):
                print("drone call function getCamCoptObj happen fail", copter_id)
                sys.exit(0)
            val.RFlySimObj = drone
            recv_ = recv_ + [val]
            for cam_id, cam_ in self.drones[copter_id].RGBCamera.items():
                cam = self.mav.getCamCoptObj(
                    0, self.drones[copter_id].RGBCamera[cam_id].seq_id)
                if not isinstance(cam, PX4MavCtrl.CameraData):
                    print("camera call function getCamCoptObj happen fail",
                          self.drones[copter_id].RGBCamera[cam_id].seq_id)
                    sys.exit(0)
                cam_.RFlySimObj = cam
                recv_ = recv_ + [cam_]
        for obj_id, val in self.objs.items():
            obj = self.mav.getCamCoptObj(1, int(obj_id))
            if not isinstance(obj, PX4MavCtrl.CoptReqData):
                print("object call function getCamCoptObj happen fail", obj_id)
                sys.exit(0)
            val.RFlySimObj = obj
            recv_ = recv_ + [val]
        # print("len recv_ ", len(recv_))
        while True:
            for obj in recv_:
                r_obj = obj.RFlySimObj
                if r_obj.hasUpdate:
                    obj.position = list(r_obj.PosUE)
                    obj.orientation = list(r_obj.angEuler)
                    obj.UpdateMatrix()
                    obj.refresh_time = r_obj.timestmp
                    if obj.class_type == 'camera':
                        if(not str(obj.seq_id) in self.rgb_camera.keys() and not str(obj.seq_id) in self.inf_camera.keys()):
                            # 这个相机不在当前配置文件之列, 不做处理
                            continue
                        obj.SetFOV(r_obj.CameraFOV)
                        obj.SetImgSize(r_obj.DataWidth, r_obj.DataHeight)
                        # print("camera ", r_obj.PosUE, r_obj.angEuler)
                        # obj.UpdateMatrix()
                        # print("posUE ", r_obj.PosUE,
                        #   "angle : ", r_obj.angEuler)
                        obj.hasUpdate = True
                        if(str(obj.id) in self.drones[str(obj.belong_copter)].infrareds.keys()):
                            # 同步更新云台上的红外相机参数
                            self.drones[str(obj.belong_copter)].infrareds[str(
                                obj.id)].position = list(r_obj.PosUE)
                            self.drones[str(obj.belong_copter)].infrareds[str(
                                obj.id)].orientation = list(r_obj.angEuler)
                            # print("cam, angle: ", r_obj.angEuler,
                            #       " degree: ", np.array(r_obj.angEuler) / math.pi * 180)
                            self.drones[str(obj.belong_copter)].infrareds[str(
                                obj.id)].UpdateMatrix()
                            self.drones[str(obj.belong_copter)].infrareds[str(
                                obj.id)].hasUpdate = True
                            # print("update camera pose")
                        continue
                    obj.bounding_box = copy.deepcopy(r_obj.BoxExtent)
                    obj.box_origin = copy.deepcopy(r_obj.boxOrigin)
                    # if len(obj.box_apex) == 0 and len(obj.bounding_box) > 0:
                    obj.CalBBoxApex()  # 更新box给顶点坐标
                    if(obj.class_type == 'drone'):  # 如果是固定翼飞机，需要做尾焰特殊处理
                        shape_ = np.array(
                            [-obj.bounding_box[0] - obj.tail_flame_len, 0, 0]) + np.array(obj.box_origin)
                        obj.box_apex.append(shape_.tolist())
                        for idx in range(int(360/obj.tail_flame_res)):
                            theta = idx*obj.tail_flame_res
                            y = math.cos(math.radians(theta)) * \
                                obj.tail_flame_rad
                            x = -obj.bounding_box[0]
                            z = -math.sin(math.radians(theta)) * \
                                obj.tail_flame_rad
                            rad_ = np.array([x, y, z]) + \
                                np.array(obj.box_origin)
                            obj.box_apex.append(rad_.tolist())
                    r_obj.hasUpdata = False

            # 因为没有使用多线程的方式，所以这里刷新频率一定要够快,与PX4MavCtrlV4采用同样的刷新频率
            time.sleep(0.001)

    def CollisionStatus(self):
        '''
        由平台输出物体的销毁情况,及时更新目标状态
        '''
        while True:
            for i in range(len(self.mav.inReqVect)):
                val = self.mav.inReqVect[i]
                if(val.hasUpdate):
                    if(val.CrashType != 0 and str(val.copterID) in self.drones.keys()):
                        for key, copter_id in self.inf_camera.items:
                            if(str(copter_id) == str(val.copterID)):
                                self.inf_camera.pop(key)
                        for key, copter_id in self.rgb_camera.items():
                            if(str(copter_id) == str(val.copterID)):
                                self.rgb_camera.pop(key)
                        self.drones.pop(str(val.copterID))

                    if(val.CrashType != 0 and str(val.copterID) in self.objs.keys()):
                        self.drones.pop(str(val.copterID))
                    val.hasUpdate = False

            time.sleep(0.001)

    def GetTargetPose(self, copter_id):
        if(str(copter_id) in self.drones.keys()):
            # if(time.time() - self.drones[str(copter_id)].refresh_time < 0.02):延迟问题在做考虑
            position = self.drones[str(copter_id)].position
            ori = self.drones[str(copter_id)].orientation
            deg = [math.degrees(ori[0]), math.degrees(
                ori[1]), math.degrees(ori[2])]
            return position, deg
            # else:
            print("get status time out")
        if(str(copter_id) in self.objs.keys()):
            # if(time.time() - self.objs[str(copter_id)].refresh_time < 0.02):
            position = self.objs[str(copter_id)].position
            ori = self.objs[str([copter_id])].orientation
            return position, [math.degrees(ori[0]), math.degrees(ori[1]), math.degrees(ori[2])]
            # else:
            print("get status time out")
        return [], []

    def GetCamera(self, copter_id, camera_id, class_type='') -> Camera:
        if str(copter_id) in self.drones.keys():
            if class_type == 'infrared' and str(camera_id) in self.drones[str(copter_id)].infrareds.keys():
                cam = self.drones[str(copter_id)].infrareds[str(camera_id)]
                return cam
            elif class_type == 'rgb' and str(camera_id) in self.drones[str(copter_id)].RGBCamera.keys():
                cam = self.drones[str(copter_id)].RGBCamera[str(camera_id)]
                return cam
            else:
                print("camera not exist")
        else:
            print('copter not exist')
        return [], []

    def GetCameraOri(self, copter_id, camera_id):
        if str(copter_id) in self.drones.keys():
            drone = self.drones[str(copter_id)]
            if str(camera_id) in drone.RGBCamera.keys():
                cam = drone.RGBCamera[str(camera_id)]
                return self.vis.VisSensor[cam.seq_id].SensorAngEular
            elif str(camera_id) in drone.infrareds.keys():
                cam = drone.infrareds[str(camera_id)]
                return cam.GetOrientation()
        else:
            print("camera not exist")
            return []

    def GetImg(self, copter_id, camera_id) -> list:
        '''
            copter_id: 需要获取相机图像的飞机
            camera_id: 需要获取的相机图像,这里的编号是当前飞机给予的编号
            为减少计算,只有在需要图像数据的时候才拷贝图像
        '''
        if str(copter_id) in self.drones.keys():
            drone = self.drones[str(copter_id)]

            # infrared = self.drones[str(copter_id)].infrareds[str(camera_id)]
            # rgb = self.drones[str(copter_id)].RGBCamera[str(camera_id)]

            # print("infrared: ", infrared.orientation)
            # print("rgb: ", rgb.orientation)

            if str(camera_id) in drone.RGBCamera.keys():
                while True:
                    if self.vis.hasData[drone.RGBCamera[str(copter_id)].seq_id]:
                        start_time = time.time()
                        # if(abs(start_time-self.vis.timeStmp[drone.RGBCamera[str(copter_id)].seq_id]) > 0.02):
                        # print("get img time out")
                        self.vis.hasData[drone.RGBCamera[str(
                            copter_id)].seq_id] = False
                        self.vis.Img_lock[drone.RGBCamera[str(
                            copter_id)].seq_id].acquire()
                        img = copy.deepcopy(
                            self.vis.Img[drone.RGBCamera[str(copter_id)].seq_id])
                        self.vis.Img_lock[drone.RGBCamera[str(
                            copter_id)].seq_id].release()
                        rgb_c = drone.RGBCamera[str(camera_id)]
                        cv2.line(img, (int(rgb_c.half_w - 1/40 * rgb_c.data_width), int(rgb_c.half_h)),
                                 (int(rgb_c.half_w+1/40 * rgb_c.data_width), int(rgb_c.half_h)), (0, 0, 255), int(rgb_c.data_width/400))
                        cv2.line(img, (int(rgb_c.half_w),
                                 int(rgb_c.half_h - 1/40 * rgb_c.data_height)),
                                 (int(rgb_c.half_w), int(rgb_c.half_h+1/40 * rgb_c.data_height)), (0, 0, 255), int(rgb_c.data_width/400))
                        return img
                    time.sleep(0.001)
            elif str(camera_id) in drone.infrareds.keys():
                # 做两次坐标转换，需要输出目标，且要判断目标是否在视场内,同时需要判断目标有没有被销毁，然后3D到2D变化
                # if time.time() - drone.infrareds[str(camera_id)].refresh_time > 0.02:
                # print("camera data time out")
                # 非fixed的相机是可以直接从平台获得位姿的
                if(str(camera_id) in drone.RGBCamera.keys() and
                        not drone.infrareds[str(camera_id)].hasUpdate):
                    print("the camera hasn't update")
                    return[]
                # 在获取数据前更新矩阵，可以减少计算
                if not drone.infrareds[str(camera_id)].UpdateMatrix():
                    print("the matrix update fial")
                    sys.exit(0)
                # drone.infrareds[str(camera_id)].hasUpdate = False
                # if drone.infrareds[str(camera_id)].joint_type == 'fixed':
                    # 刚性连接相机目标计算,
                drone.UpdateMatrix()
                # trans_matrix = trans_matrix @ drone.trans_R
                img_w = drone.infrareds[str(camera_id)].data_width
                img_h = drone.infrareds[str(camera_id)].data_height
                drone_ptx = []
                ret = []
                for drone_id, drone_ in self.drones.items():
                    if(drone_id == str(copter_id)):  # 不考虑自身的成像
                        continue
                    # if(time.time() - drone_.refresh_time > 0.02):
                        # print("get obj status time out")
                    # drone_.CalBBoxApex()
                    points = self.ObjVehicle2World(
                        drone_.position, drone_.orientation, drone_.box_apex)
                    points = np.insert(
                        points, 0, [[drone_.box_origin[0]+drone_.position[0], drone_.box_origin[1]+drone_.position[1], drone_.box_origin[2] + drone_.position[2]]], axis=0)
                    drone_ptx = drone_ptx + points.tolist()
                if len(drone_ptx) > 0:
                    drone_img = self.TransformObjs(
                        drone.trans_R, drone.infrareds[str(camera_id)].trans_R, drone.infrareds[str(camera_id)].internal_matrix, drone.infrareds[str(camera_id)].position, drone.position, drone_ptx, drone.infrareds[str(camera_id)].is_from_rflysim, 0)
                    ret = self.Imaging('drone', drone.infrareds[str(camera_id)], [], drone_img, img_w,
                                       img_h, drone.tail_flame_res)
                obj_ptx = []
                obj_seqId = []
                for obj_id, obj in self.objs.items():
                    # if(time.time() - obj.refresh_time > 0.02):
                    # print("get obj status time out")
                    obj.CalBBoxApex()
                    points = self.ObjVehicle2World(
                        obj.position, obj.orientation, obj.box_apex)
                    points = np.insert(
                        points, 0, [[obj.box_origin[0]+obj.position[0], obj.box_origin[1] + obj.position[1], obj.box_origin[2] + obj.position[2]]], axis=0)
                    obj_ptx = obj_ptx + points.tolist()
                    obj_seqId = obj_seqId + [int(obj_id)]
                if(len(obj_ptx) > 0):
                    obj_img = self.TransformObjs(
                        drone.trans_R, drone.infrareds[str(camera_id)].trans_R, drone.infrareds[str(camera_id)].internal_matrix, drone.infrareds[str(camera_id)].position, drone.position, obj_ptx, drone.infrareds[str(camera_id)].is_from_rflysim, 0)
                    ret = ret + \
                        self.Imaging('obj', drone.infrareds[str(
                            camera_id)], obj_seqId, obj_img, img_w, img_h)

                # 给图像添加噪声
                gauss = (np.random.normal(drone.infrareds[str(
                    camera_id)].mean, drone.infrareds[str(camera_id)].sigma, (img_h, img_w))).astype(np.int8)
                img = np.zeros((img_h, img_w), np.int8)
                img = (img + gauss)
                img = np.clip(img, a_min=0, a_max=255).astype(np.uint8)
                if(len(ret) > 0):
                    for pts in ret:
                        hull = cv2.convexHull(np.array(pts).astype(
                            np.int32), returnPoints=True)
                        cv2.fillConvexPoly(img, hull, (255, 255, 255))
                # cv2.imshow("test", img)
                # cv2.waitKey()
                return img

            else:
                print("not exist camera")
        else:
            print("not exist drone")
        return []

    def TransformCoordinate(self, matrix: np.matrix,  pos: np.matrix) -> np.array:
        '''
            matrix:4x4 变换矩阵 或是
            pos: 4x1矩阵
            reutrn: 4x1矩阵
        '''
        return matrix @ pos
        pass

    def TransformObjs(self, Rotate_V_W: np.matrix, Rotae_C_V: np.matrix, cam_internal_matrix: np.matrix, cam_pos, drone_pos, bbox: list, isRflyObj, trans_flag) -> list:
        '''
            extern_matrix: 世界坐标系外参旋转矩阵,4x4矩阵
            cam_internal_matrix: 相机的内参矩阵 3x4 矩阵
            bbox: 相对目标的顶点坐标,其中如果目标是固定飞机,还包括尾焰信息,nx4 list
            trans_flag: 0: 转换3D, 1:转换2D,2:两者都需要
            isRflyObj: 相机是否来自RflySim,下视云台视为来自RflySim
            返回需要转换的数据
        '''
        if(len(Rotae_C_V) == 0 or len(Rotate_V_W) == 0 or len(cam_internal_matrix) == 0 or len(cam_pos) == 0 or len(drone_pos) == 0):
            print('have data error !')
            sys.exit(0)

        if(len(bbox) == 0):
            # 说明没有目标在视野里
            return []

        sensor_coordinate = []
        if(not isRflyObj):
            vehicle_coorinate = Rotate_V_W.T @ (
                np.array(bbox) - np.array(drone_pos)).T
            tmp = np.array(cam_pos).reshape(3, 1)
            sensor_coordinate = Rotae_C_V.T @ (
                vehicle_coorinate - tmp)
        else:
            # 直接可以获取到全局坐标系下的相机坐标
            sensor_coordinate = Rotae_C_V.T @ (
                np.array(bbox) - np.array(cam_pos)).T

        # if(pos.shape[0] < 3):
        #     pos = np.insert(pos, pos.shape[0], values=np.ones((
        #         1, pos.shape[1])), axis=0)  # 4xn矩阵

        # image_coordinate = (cam_internal_matrix @
        #                     sensor_coordinate)
        fx = cam_internal_matrix.item(0)
        fy = cam_internal_matrix.item(5)
        cx = cam_internal_matrix.item(2)
        cy = cam_internal_matrix.item(6)
        xc = sensor_coordinate[0, :]
        yc = sensor_coordinate[1, :]
        zc = sensor_coordinate[2, :]
        x = np.multiply(fx/xc, yc) + cx
        y = np.multiply(fy/xc, zc) + cy

        two_d = np.vstack((x, y)).T  # 沿着矩阵列拼接

        # tow_dim = np.delete(image_coordinate.T.tolist(), 2, axis=1)
        # two_dim =
        if(trans_flag == 0):
            # 输出传感器坐标系下的点坐标
            return sensor_coordinate.T.tolist()
        elif(trans_flag == 1):
            # 输出，图像坐标系下点
            return two_d.tolist()
        else:
            # 合并输出时，三维数据在前，二维数据在后，两个数量相等，维度不等
            return sensor_coordinate.T.tolist() + two_d

    def CtrlCameraRat(self, copter_id, cam_id, angEuler: list):
        '''
            控制某架飞机上的某一个相机旋转
            copter_id: 飞机在RFlySim中唯一的标识
            cam_d: 相机在飞机上的编号
            angEuler:需要旋转的角度[roll,pith,yaw],单位和平台统一使用角度制
        '''
        # 这里只转化命令，至于相机旋转约束，由外部决策模块设置

        if(not str(copter_id) in self.drones):
            print("not exist ", copter_id, " drone")
            sys.exit(0)
        else:
            val = self.drones[str(copter_id)]
            if(str(cam_id) in val.RGBCamera.keys() and str(cam_id) in val.infrareds.keys()):
                cam_rgb = val.RGBCamera[str(cam_id)]
                infrared = val.infrareds[str(cam_id)]
                # cam_rgb.SetOrientation(angEuler) #可见光的相机角度有平台给出
                # infrared.SetOrientation(angEuler)
                vis = self.vis.VisSensor[cam_rgb.seq_id]
                vis.SensorAngEular = copy.deepcopy(angEuler)
                # print("angle ", angEuler)
                self.vis.sendUpdateUEImage(vis)
            elif(str(cam_id) in val.infrareds.keys()):
                cam = val.infrareds[str(cam_id)]
                cam.SetOrientation(angEuler)
            else:
                print("the camera not exist")

        # [math.radians(angEuler[0]), math.radians(
        #     angEuler[1]), math.radians(angEuler[2])]
        # self.vis.sendUpdateUEImage(
            # [math.radians(angEuler[0]), math.radians(angEuler[1]), math.radians(angEuler[2])])

    def SetCameraProperty(self, copter_id, cam_id, FOV=0, pos=[], ori=[], img_w=0, img_h=0, class_type='infrared'):
        '''
        copter_id: Rflysim中飞机的copter_id,
        cma_id: 在飞机上的相机编号,不是配置文中的seq_id
        ori:角度制
        FOV:角度
        ...
        '''
        if(not str(copter_id) in self.drones):
            print("not exist ", copter_id, " drone")
            sys.exit(0)
        else:
            val = self.drones[str(copter_id)]
            if(str(cam_id) in val.RGBCamera.keys() and str(cam_id) in val.infrareds.keys()):
                cam_rgb = val.RGBCamera[str(cam_id)]
                infrared = val.infrareds[str(cam_id)]
                vis = self.vis.VisSensor[cam_rgb.seq_id]
                if(len(pos) > 0):
                    cam_rgb.SetPosition(pos)
                    infrared.SetPosition(pos)
                    vis.SensorPosXYZ = copy.deepcopy(pos)
                if(len(ori) > 0):

                    rpy = [math.radians(ori[0]), math.radians(
                        ori[1]), math.radians(ori[2])]
                    cam_rgb.SetOrientation(rpy)
                    infrared.SetOrientation(rpy)
                    vis.SensorAngEular = copy.deepcopy(rpy)

                if(class_type == 'rgb'):
                    if(FOV > 0):
                        cam_rgb.SetFOV(FOV)
                        vis.CameraFOV = FOV
                    if(img_w > 0 and img_h > 0):
                        vis.DataWidth = img_w
                        vis.DataHeight = img_h
                        cam_rgb.SetImgSize(int(img_w), int(img_h))
                elif(class_type == 'infrared'):
                    if(FOV > 0):
                        infrared.SetFOV(FOV)
                    if(img_w > 0 and img_h > 0):
                        infrared.SetImgSize(int(img_w), int(img_h))

                self.vis.sendUpdateUEImage(vis)
            elif str(cam_id) in val.infrareds.keys():
                infrared = val.infrareds[str(cam_id)]
                if(len(pos) > 0):
                    infrared.SetPosition(pos)
                if(len(ori) > 0):
                    infrared.SetOrientation(ori)
                if(FOV > 0):
                    infrared.SetFOV(FOV)
                if(img_w > 0 and img_h > 0):
                    infrared.SetImgSize(int(img_w), int(img_h))

    def SetCameraFOV(self, copter_id, cam_id, FOV, class_type='infrared'):
        if str(copter_id) in self.drones.keys():
            val = self.drones[str(copter_id)]
            if(class_type == 'rgb' and str(cam_id) in self.drones[str(copter_id)].RGBCamera.keys()):
                # 需要设置RFlySim参数
                cam_rgb = val.RGBCamera[str(cam_id)]
                cam_rgb.SetFOV(FOV)
                vis = self.vis.VisSensor[cam_rgb.seq_id]
                # vis.SensorAngEular = copy.deepcopy(cam_rgb.orientation)
                vis.CameraFOV = FOV
                self.vis.sendUpdateUEImage(vis)
            elif class_type == 'infrared' and str(cam_id) in val.infrareds.keys():
                val.infrareds[str(cam_id)].SetFOV(FOV)
            else:
                print("not exist camera")
        else:
            print("not exist drone")

    def Imaging(self, class_type: str, camera: Camera, objs, pos: list, img_w: int, img_h: int, tail_flame_res=10,) -> list:
        '''
            pos[0] : obj img_coordinate
            pos[1-9] : box aptex img_coordinate
            如果目标是飞机,pos[10-10+int(360/tail_flame_res)]尾焰数据
            camera 需要获取的具体相机
        '''
        ret = []
        fx = camera.internal_matrix.item(0)
        fy = camera.internal_matrix.item(5)
        cx = camera.internal_matrix.item(2)
        cy = camera.internal_matrix.item(6)
        # xc = sensor_c[0, :]
        # yc = sensor_c[1, :]
        # zc = sensor_c[2, :]
        # x = np.multiply(fx/xc, yc) + cx
        # y = np.multiply(fy/xc, zc) + cy

        if class_type == 'drone':
            tail_offset = int(360/tail_flame_res) + 1  # 算上尾焰锥体的顶点
            for i in range(0, len(pos), tail_offset+9):
                obj = np.array(pos[i:i+tail_offset+9]).T
                xc = obj[0, :]
                if np.any(xc < 0):  # 在坐标系x负方向都应该剔除
                    continue
                yc = obj[1, :]
                zc = obj[2, :]
                x = np.multiply(fx/xc, yc) + cx
                y = np.multiply(fy/xc, zc) + cy
                points = np.vstack((x, y)).T
                obj = points.tolist()
                if(obj[0][0] < 0 or obj[0][1] < 0 or obj[0][0] >= img_w or obj[0][1] >= img_h):
                    # 如果目标中心位置不在图像内，不做成像处理
                    continue
                tial = obj[-tail_offset:]
                for p in tial:
                    if p[0] < 0 or p[1] < 0 or p[0] >= img_w or p[1] >= img_h:
                        break
                # 这里就是需要显示的内容
                # hull = cv2.convexHull(
                #     np.array(tial).astype(np.int32), returnPoints=True)
                # print(int(i / 46))
                # cv2.fillConvexPoly(
                #     img, hull, (255, 255, 255))
                # cv2.imshow("test", img)
                # cv2.waitKey(0)
                ret = ret+[[tial]]
        else:
            # 普通目标
            camera.insight_obj = []
            for i in range(0, len(pos), 9):
                # obj = pos[i:i+9]
                obj = np.array(pos[i:i+9]).T
                xc = obj[0, :]
                if np.any(xc < 0):  # 在坐标系x负方向都应该剔除
                    continue
                yc = obj[1, :]
                zc = obj[2, :]
                x = np.multiply(fx/xc, yc) + cx
                y = np.multiply(fy/xc, zc) + cy
                points = np.vstack((x, y)).T
                obj = points.tolist()

                min_x = 1000000000
                min_y = 1000000000
                max_x = -1
                max_y = -1
                for k in range(len(obj)):
                    if obj[0][0] < 0 or obj[0][1] < 0 or obj[0][0] >= img_w or obj[0][1] >= img_h:
                     # 如果目标中心位置不在图像内，不做成像处理
                        break
                    else:
                        if(obj[k][0] < min_x):
                            min_x = obj[k][0]
                        if(obj[k][1] < min_y):
                            min_y = obj[k][1]
                        if(obj[k][0] > max_x):
                            max_x = obj[k][0]
                        if(obj[k][1] > max_y):
                            max_y = obj[k][1]
                if(max_x < 0 or max_y < 0 or min_x > img_w or min_y > img_h):
                    break
                if(max_x >= img_w):
                    max_x = img_w-1
                if(max_y >= img_h):
                    max_y = img_h - 1
                if(min_x < 0):
                    min_x = 0
                if(min_y < 0):
                    min_y = 0
                camera.insight_obj = camera.insight_obj + \
                    [objs[int(i / 9)]]  # 当前目标再视场角内
                ret = ret + [[[min_x, min_y], [max_x, min_y],
                              [max_x, max_y], [min_x, max_y]]]

        return ret

    def ObjVehicle2World(self, pos, ori, points: list) -> list:
        '''
            从载体坐标系转换到世界坐标系下
            pos:载体在全局坐标系下的位置
            ori:载体在世界坐标系下的欧拉角(unit:degree))
            points: [[x,y,z],[x,y,z]].... 需要转换的载体坐标系下的点
            返回n x 3 坐标list

        '''
        if(len(points) == 0 or len(pos) == 0 or len(ori) == 0):
            return[]
        rpy = [math.radians(ori[0]), math.radians(
            ori[1]), math.radians(ori[2])]
        cos_r = math.cos(rpy[0])
        cos_p = math.cos(rpy[1])
        cos_y = math.cos(rpy[2])
        sin_r = math.sin(rpy[0])
        sin_p = math.sin(rpy[1])
        sin_y = math.sin(rpy[2])
        trans_matrix = np.matrix([[cos_p*cos_y, sin_r*sin_p*cos_y - cos_r*sin_y, cos_r*sin_p*cos_y + sin_r*sin_y, pos[0]],
                                  [cos_p*sin_y, sin_r*sin_p*sin_y + cos_r*cos_y,
                                 cos_r*sin_p*sin_y - sin_r*cos_y, pos[1]],
                                  [-sin_p, sin_r*cos_p, cos_r *
                                 cos_p, pos[2]],
                                  [0, 0, 0, 1]])
        points_set = np.matrix(points).T
        points_set = np.insert(points_set, points_set.shape[0], values=np.ones((
            1, points_set.shape[1])), axis=0)  # 4xn矩阵

        world_points = trans_matrix @ points_set
        return np.delete(world_points, 3, 0).T.tolist()

    def DetectObj(self, img):
        '''

        '''

        pass

    def AimWeapon(self, copter_id, camera_id) -> bool:
        '''
            武器锁定目标,这里只有个调用了GetImg 才会更新数据
            mode: 默认为自动锁定目标
            cmd: 可以通过命令取消锁定
        '''
        # if (not hasattr(self.AimWeapon, 'seq_id', 'copter_id', 'camera_id')):
        #     self.AimWeapon.seq_id = -1
        #     self.AimWeapon.copter_id = copter_id
        #     self.AimWeapon.camera_id = camera_id

        if str(copter_id) in self.drones.keys():
            drone = self.drones[str(copter_id)]
            if(str(camera_id) in drone.infrareds.keys() and str(camera_id) in drone.RGBCamera.keys()):
                cam = self.drones[str(copter_id)].infrareds[str(camera_id)]
                if(not cam.is_from_rflysim):
                    return False
                if(not len(cam.insight_obj) == 1):
                    # 视场内只会有一个目标出现
                    return False
                else:
                    if str(cam.insight_obj[0] in self.objs.keys()):
                        obj = self.objs[str(cam.insight_obj[0])]
                        pos = list(np.array(obj.box_origin) +
                                   np.array(obj.position))
                        # pos = self.ObjVehicle2World(
                        #     obj.position, obj.orientation, obj.box_apex)
                        # pos = np.insert(
                        #     pos, 0, [[obj.box_origin[0], obj.box_origin[1], obj.box_origin[2]]], axis=0)

                        # dist = np.linalg.norm(
                        #     np.array(pos) - np.array(cam.position))
                        while True:
                            ret = self.TransformObjs(drone.trans_R, cam.trans_R, cam.internal_matrix,
                                                     cam.position, drone.position, [pos], True, 1)
                            x = ret[0][0]
                            y = ret[0][1]
                            dx = x - cam.half_w
                            dy = y - cam.half_h
                            vis = self.vis.VisSensor[drone.RGBCamera[str(
                                camera_id)].seq_id]
                            f = cam.internal_matrix[0, 0]
                            theta = -math.radians(vis.SensorAngEular[0])
                            ex = math.cos(theta)*dx + math.sin(theta)*dy
                            ey = math.cos(theta)*dy - math.sin(theta)*dx
                            pitch = math.atan(ex/f)
                            yaw = math.atan(ex/f)
                            if abs(dx) < 3 and abs(dy) < 3:
                                # return SensorAngEular
                                break
                            if abs(ex) > cam.half_w:
                                if ex > 0:
                                    ex = cam.half_w
                                else:
                                    ex = -cam.half_w
                            if abs(ey) > cam.half_h:
                                if ey > 0:
                                    ey = cam.half_h
                                else:
                                    ey = -cam.half_h

                            pitch = vis.SensorAngEular[1] + \
                                math.degrees(math.atan(-ey/f))
                            yaw = vis.SensorAngEular[2] + \
                                math.degrees(math.atan(ex/f))

                            if(pitch < math.degrees(cam.min_rotation[1]) or pitch > math.degrees(cam.max_rotation[1]) or yaw < math.degrees(cam.min_rotation[2]) or yaw > math.degrees(cam.max_rotation[2])):
                                print("current rotation over constraint condition")
                                return False
                            tran = [vis.SensorAngEular[0], pitch, yaw]
                            vis.SensorAngEular = list(tran)
                            self.vis.sendUpdateUEImage(vis)
                            time.sleep(0.1)
                        return True
                    else:
                        print('Aim weapon fail, the object not exist')
                        sys.exit(0)
            else:
                print("no exist camera")
                sys.exit(0)
        else:
            print("no exist drone")
            pass

    def StartCaptureImg(self):

        recv_th = threading.Thread(target=self.RecvRflySimInfo)
        recv_th.start()
        time.sleep(2)

        for infrared_id, drone_id in self.inf_camera.items():
            # infrared.
            th = threading.Thread(target=self.GenData,
                                  args=(infrared_id, self.drones[str(drone_id)].infrareds[str(infrared_id)].rate, self.vis))
            th.start()

    def GenData(self, SeqID, rate, vis: VisionCaptureApi.VisionCaptureApi):

        topic_name = "/rflysim/sensor" + str(SeqID) + "/img_infrared"
        inter_val = 1/rate
        lastTime = time.time()
        type = None
        pub = None
        if(isEnableRosTrans and isLinux):
            type = sensor.Image
            pub = rospy.Publisher(topic_name, type, queue_size=10)
        while True:
            lastTime = lastTime + inter_val
            sleepTime = lastTime - time.time()
            if sleepTime > 0:
                time.sleep(sleepTime)
            else:
                lastTime = time.time()

            if str(SeqID) in self.inf_camera.keys():
                vis.hasData[int(SeqID)] = False
                img = self.GetImg(
                    self.inf_camera[str(SeqID)], SeqID)
                vis.Img[int(SeqID)] = copy.deepcopy(img)
                vis.hasData[int(SeqID)] = True
                if(isEnableRosTrans and isLinux):
                    header = std_msg.Header()
                    header.frame_id = "infrared"
                    header.stamp = rospy.Time.now()
                    encoding_ = "mono8"
                    msg = sensor.Image()
                    msg.height = img.shape[0]
                    msg.width = img.shape[1]
                    msg.encoding = encoding_
                    msg.data = img.tostring()
                    msg.step = msg.width * 1
                    pub.publish(msg)
            else:  # 飞机销毁了，附在飞机上的相机也就随之销毁
                break
            time.sleep(inter_val)
        pass
