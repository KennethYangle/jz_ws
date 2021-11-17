#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import os
import json
import rospy, rospkg
import cv2
from geometry_msgs.msg import TwistStamped, PoseStamped
from swarm_msgs.msg import Action, BoundingBoxes


class Allocation:

    def __init__(self, drone_id, drone_num, params):
        self.drone_id = drone_id
        self.drone_num = drone_num
        self.R_cb = np.array([[0,0,1], [-1,0,0], [0,-1,0]])
        self.T_ce_dic = dict()
        self.f = params["F"]
        self.u0 = params["WIDTH"] / 2
        self.v0 = params["HEIGHT"] / 2
        self.K = np.array([[self.f, 0, self.u0], [0, self.f, self.v0], [0, 0, 1]])
        self.th_Sam = 5e-4  # 太小会多判出目标；太大会合并目标，最好要base飞机看到所有目标，能保证的话取大一些更好
        self.start_time = time.time()
        
        # 依次储存位置、R、是否初始化
        self.mav_pos_dic = dict()
        self.mav_R_dic = dict()
        self.img_pos_dic = dict()
        self.R_ec_dic = dict()
        self.features_info = dict()
        self.is_initialized_poses = [False for i in range(self.drone_num)]
        self.is_initialized_imges = [False for i in range(self.drone_num)]
        self.dj_action = Action()

        # Subscribers and Publishers
        self.mav_pos_subs = [None for i in range(self.drone_num)]
        self.pos_image_subs = [None for i in range(self.drone_num)]
        for i in range(self.drone_num):
            self.mav_pos_subs[i] = rospy.Subscriber("drone_{}/mavros/local_position/pose_cor".format(i+1), PoseStamped, self.mav_pose_cb, i)
        for i in range(self.drone_num):
            self.pos_image_subs[i] = rospy.Subscriber("drone_{}/tracker/pos_image".format(i+1), BoundingBoxes, self.pos_image_cb, i)
        self.Expect_action_pub = rospy.Publisher('expect_action'+str(self.drone_id), Action, queue_size=10)


    def mav_pose_cb(self, msg, args):
        id = args
        drone_name = "drone_{}".format(id+1)
        self.is_initialized_poses[id] = True

        mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.mav_pos_dic[drone_name] = mav_pos
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        mav_R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
        self.mav_R_dic[drone_name] = mav_R


    def pos_image_cb(self, msg, args):
        id = args
        drone_name = "drone_{}".format(id+1)
        self.is_initialized_imges[id] = True

        objects = dict()
        for bbox in msg.bounding_boxes:
            x = (bbox.xmin + bbox.xmax) / 2
            y = (bbox.ymin + bbox.ymax) / 2
            objects[bbox.id] = [x, y]
        self.img_pos_dic[drone_name] = objects

    def check_ready(self):
        # print("is_initialized_poses: {}".format(self.is_initialized_poses))
        # print("is_initialized_imges: {}".format(self.is_initialized_imges))
        is_ready = True
        for init in self.is_initialized_poses:
            is_ready = is_ready and init
        for init in self.is_initialized_imges:
            is_ready = is_ready and init
        return is_ready

    def Sampson(self, R0, R1, T0, T1, pp0, pp1):
        def skew(v):
            return np.array([[0, -v[2,0], v[1,0]], 
                             [v[2,0], 0, -v[0,0]], 
                             [-v[1,0], v[0,0], 0]])

        R_c1c0 = R0.dot(R1.T)
        T_c1c0 = R0.dot(T1 - T0)
        F = np.dot(skew(T_c1c0), R_c1c0)
        pi0 = np.array(pp0+[1]).reshape((-1,1))
        pi1 = np.array(pp1+[1]).reshape((-1,1))
        # print(pi0, pi1)
        p0 = np.linalg.inv(self.K).dot(pi0)
        p1 = np.linalg.inv(self.K).dot(pi1)

        num = (p0.T.dot(F).dot(p1))[0,0] ** 2
        a = p0.T.dot(F)
        b = F.dot(p1)
        den = a[0,0]**2 + a[0,1]**2 + b[0,0]**2 + b[1,0]**2
        return num/den

    def match_features(self):
        # 选base无人机——特征最多的
        max_feat_num = 0
        for drone_name, features in self.img_pos_dic.items():
            if len(features) > max_feat_num:
                max_feat_num = len(features)
                base_name = drone_name
                base_features = features
        
        # 创建以特征为主体的数据结构
        for name, mav_R in self.mav_R_dic.items():
            R_ec = (mav_R.dot(self.R_cb)).T
            self.R_ec_dic[name] = R_ec
        for name, mav_pos in self.mav_pos_dic.items():
            T_ce = mav_pos.reshape((-1,1))
            self.T_ce_dic[name] = T_ce

        for id, pos_i in base_features.items():
            self.features_info[id] = dict()
            self.features_info[id]["matched_pos_i_list"] = [pos_i]
            self.features_info[id][base_name] = id
            self.features_info[id]["R_ec_list"] = [self.R_ec_dic[base_name]]
            self.features_info[id]["T_ce_list"] = [self.T_ce_dic[base_name]]
            self.features_info[id]["M_list"] = [self.K.dot(self.R_ec_dic[base_name]).dot( np.hstack((np.identity(3), -self.T_ce_dic[base_name])) )]

        # 循环其他飞机
        for drone_name, features in self.img_pos_dic.items():
            if drone_name == base_name: continue
            # 尝试匹配base_features和drone_features
            for bf_id, bf_fe in base_features.items():
                min_Sam = 0x3f3f3f3f
                for df_id, df_fe in features.items():
                    sampson_distance = self.Sampson(self.R_ec_dic[base_name], self.R_ec_dic[drone_name], 
                        self.T_ce_dic[base_name], self.T_ce_dic[drone_name], bf_fe, df_fe)
                    print("{}-{} sampson: {}".format(bf_id, df_id, sampson_distance))
                    if sampson_distance < self.th_Sam and sampson_distance < min_Sam:
                        min_Sam = sampson_distance
                        best_fe = df_fe
                        best_id = df_id

                if min_Sam < self.th_Sam:
                    self.features_info[bf_id]["matched_pos_i_list"].append(best_fe)
                    self.features_info[bf_id][drone_name] = best_id
                    self.features_info[bf_id]["R_ec_list"].append(self.R_ec_dic[drone_name])
                    self.features_info[bf_id]["T_ce_list"].append(self.T_ce_dic[drone_name])
                    self.features_info[bf_id]["M_list"].append( self.K.dot(self.R_ec_dic[drone_name]).dot( np.hstack((np.identity(3), -self.T_ce_dic[drone_name])) ) )
        print("features_info: {}".format(self.features_info))

    def reconstruction(self):
        for fe_id, fe_info in self.features_info.items():
            srcA, srcb = [], []
            for i in range(len(fe_info["matched_pos_i_list"])):
                xi, yi = fe_info["matched_pos_i_list"][i][0], fe_info["matched_pos_i_list"][i][1]
                M = fe_info["M_list"][i]
                srcA.append([M[2,0]*xi-M[0,0], M[2,1]*xi-M[0,1], M[2,2]*xi-M[0,2]])
                srcA.append([M[2,0]*yi-M[1,0], M[2,1]*yi-M[1,1], M[2,2]*yi-M[1,2]])
                srcb.append([M[0,3]-M[2,3]*xi])
                srcb.append([M[1,3]-M[2,3]*yi])
            if len(srcA) >= 4:
                ret, dstP = cv2.solve(np.array(srcA), np.array(srcb), flags=cv2.DECOMP_SVD)
                if not ret:
                    print("Solve Failed!!!")
                fe_info["pos_3d"] = dstP.flatten()
        print("features_info_3d: {}".format(self.features_info))





    def begin_task(self):
        rate = rospy.Rate(20)
        is_ready = self.check_ready()

        # wait until all ready
        while not is_ready:
            is_ready = self.check_ready()
            rate.sleep()

        # debug
        print("mav_pos_dic: {}".format(self.mav_pos_dic))
        print("mav_R_dic: {}".format(self.mav_R_dic))
        print("img_pos_dic: {}".format(self.img_pos_dic))

        # 匹配
        self.match_features()
        
        # 三维重建
        self.reconstruction()

        # 目标分配

        # 写入dj_action

        # publish dj_action
        while not rospy.is_shutdown():
            self.Expect_action_pub.publish(self.dj_action)
            rate.sleep()


if __name__=="__main__":
    rospy.init_node("allocation", anonymous=True)
    drone_id = int(rospy.get_param("~drone_id"))
    drone_num = int(rospy.get_param("~drone_num"))

    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)
    MODE = setting["MODE"]
    if MODE == "RealFlight":
        ass = Allocation(drone_id, drone_num, setting["RealFlight"])
    elif MODE == "Simulation":
        ass = Allocation(drone_id, drone_num, setting["Simulation"])
    else:
        raise Exception("Invalid MODE!", MODE)
    ass.begin_task()
