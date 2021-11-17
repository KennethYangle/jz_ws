#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped, PoseStamped
from swarm_msgs.msg import Action, BoundingBoxes


class Allocation:

    def __init__(self, param_id):
        self.start_time = time.time()
        
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_yaw = 0
        self.mav_R = np.identity(3)
        self.dj_action = Action()
        self.objects = dict()

        self.mav_pos_sub = rospy.Subscriber("mavros/local_position/pose_cor", PoseStamped, self.mav_pose_cb)
        self.pos_image_sub = rospy.Subscriber("tracker/pos_image", BoundingBoxes, self.pos_image_cb)
        self.Expect_action_pub = rospy.Publisher('expect_action'+str(param_id), Action, queue_size=10)


    def mav_pose_cb(self, msg):
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.mav_yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
        self.mav_R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

    def pos_image_cb(self, msg):
        self.objects = dict()
        for bbox in msg.bounding_boxes:
            x = (bbox.xmin + bbox.xmax) / 2
            y = (bbox.ymin + bbox.ymax) / 2
            self.objects[bbox.id] = [x, y]
        if bool(self.objects):
            self.dj_action.dj = True
            self.dj_action.id = 0
        else:
            self.dj_action.dj = False
            self.dj_action.id = 0


    def begin_task(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.Expect_action_pub.publish(self.dj_action)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("allocation", anonymous=True)
    param_id = rospy.get_param("~drone_id")

    ass = Allocation(param_id)
    ass.begin_task()