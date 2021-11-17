#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import mavros
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from swarm_msgs.msg import Action, BoundingBoxes


class Allocation:

    def __init__(self, drone_id, drone_num):
        self.drone_id = drone_id
        self.drone_num = drone_num
        self.start_time = time.time()
        
        # 依次储存位置、R、是否初始化
        self.mav_pos_list = [np.array([0., 0., 0.]) for i in range(self.drone_num)]
        self.mav_R_list = [np.identity(3) for i in range(self.drone_num)]
        self.img_pos_dic = dict()
        self.is_initialized_poses = [False for i in range(self.drone_num)]
        self.is_initialized_imges = [False for i in range(self.drone_num)]
        self.dj_action = Action()

        # Subscribers and Publishers
        self.mav_pos_subs = [None for i in range(self.drone_num)]
        self.pos_image_subs = [None for i in range(self.drone_num)]
        for i in range(self.drone_num):
            self.mav_pos_subs[i-1] = rospy.Subscriber("drone_{}/mavros/local_position/pose_cor".format(i+1), PoseStamped, self.mav_pose_cb, i)
        for i in range(self.drone_num):
            self.pos_image_subs = rospy.Subscriber("drone_{}/tracker/pos_image".format(i+1), BoundingBoxes, self.pos_image_cb, i)
        self.Expect_action_pub = rospy.Publisher('expect_action'+str(self.drone_id), Action, queue_size=10)


    def mav_pose_cb(self, msg, args):
        id = args
        self.is_initialized_poses[id] = True

        mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.mav_pos_list[id] = mav_pos
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        mav_R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
        self.mav_R_list[id] = mav_R


    def pos_image_cb(self, msg, args):
        id = args
        drone_name = "drone_{}".format(id)
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


    def begin_task(self):
        rate = rospy.Rate(20)
        is_ready = self.check_ready()

        # wait until all ready
        while not is_ready:
            is_ready = self.check_ready()
            rate.sleep()

        # debug
        print("mav_pos_list: {}".format(self.mav_pos_list))
        print("mav_R_list: {}".format(self.mav_R_list))
        print("img_pos_dic: {}".format(self.img_pos_dic))

        # publish dj_action
        while not rospy.is_shutdown():
            self.Expect_action_pub.publish(self.dj_action)
            rate.sleep()


if __name__=="__main__":
    rospy.init_node("allocation", anonymous=True)
    drone_id = int(rospy.get_param("~drone_id"))
    drone_num = int(rospy.get_param("~drone_num"))

    ass = Allocation(drone_id, drone_num)
    ass.begin_task()