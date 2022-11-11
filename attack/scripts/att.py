#!/usr/bin/env python
#coding=utf-8

import rospy, rospkg
import os
import json
import numpy as np
import math
import time
import threading
from swarm_msgs.msg import Action
from swarm_msgs.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import *
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Empty
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import SetMavFrame
from mavros_msgs.msg import State, RCIn, HomePosition
from mavros_msgs.msg import Thrust
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Image
from utils_att import Utils


mav_pos = [0, 0, 0]
mav_original_angle = [0, 0, 0]
mav_vel = np.array([0, 0, 0])
mav_yaw = 0
mav_R = np.zeros((3,3))
pos_i = [0, 0, 0, 0, 0]
image_failed_cnt = 0
command = AttitudeTarget()
dj_action = Action()


def spin():
    rospy.spin()

def mav_pose_cb(msg):
    global mav_pos, mav_yaw, mav_R, mav_pitch, mav_roll
    mav_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
    mav_yaw = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
    mav_pitch = math.asin(2*(q0*q2 - q1*q3))
    mav_roll = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1 + q2*q2))
    R_ae = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
                      [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
                      [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])
    R_ba = np.array([[0,1,0], [-1,0,0], [0,0,1]]) #mavros_coordinate to baselink_coordinate  // body to enu  # body: right-front-up3rpos_est_body)
    mav_R = R_ae.dot(R_ba)

def mav_vel_cb(msg):
    global mav_vel
    mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

def pos_image_cb(msg):
    global pos_i, image_failed_cnt, dj_action
    x, y = 0, 0
    if dj_action.dj == True:
        expect_id = dj_action.id
        for bbox in msg.bounding_boxes:
            if bbox.id == expect_id:
                x = (bbox.xmin + bbox.xmax) / 2
                y = (bbox.ymin + bbox.ymax) / 2
        # 只有一个？打他就对了
        if len(msg.bounding_boxes) == 1:
            bbox = msg.bounding_boxes[0]
            x = (bbox.xmin + bbox.xmax) / 2
            y = (bbox.ymin + bbox.ymax) / 2


    # 滤波。短时丢失还保持上一次的值，超时则丢失目标
    if x <= 0:
        image_failed_cnt += 1
    else:
        image_failed_cnt = 0
    if image_failed_cnt <= 2 and image_failed_cnt > 0:      # 20有点长
        pass
    else:
        pos_i[0] = x
        pos_i[1] = y
    # print("pos_i: {}".format(pos_i))

def expect_action_cb(msg):
    global dj_action
    dj_action = msg


if __name__=="__main__":
    src_path = os.path.join(rospkg.RosPack().get_path("offboard_pkg"), "..")
    setting_file = open(os.path.join(src_path, "settings.json"))
    setting = json.load(setting_file)
    print(json.dumps(setting, indent=4))

    MODE = setting["MODE"]
    if MODE == "RealFlight":
        u = Utils(setting["RealFlight"])
        image_center = [setting["Utils"]["WIDTH"] / 2.0, setting["Utils"]["HEIGHT"] / 2.0]
    elif MODE == "Simulation":
        u = Utils(setting["Simulation"])
        image_center = [setting["Simulation"]["WIDTH"] / 2.0, setting["Simulation"]["HEIGHT"] / 2.0]
    else:
        raise Exception("Invalid MODE!", MODE)

    rospy.init_node('attack', anonymous=True)
    spin_thread = threading.Thread(target=spin)
    spin_thread.start()
    param_id = rospy.get_param("~drone_id")

    rospy.Subscriber("mavros/local_position/pose", PoseStamped, mav_pose_cb)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, mav_vel_cb)
    rospy.Subscriber("tracker/pos_image", BoundingBoxes, pos_image_cb)
    rospy.Subscriber('expect_action'+str(param_id), Action, expect_action_cb)

    local_vel_pub = rospy.Publisher('DJ_cmd', AttitudeTarget, queue_size=10)
    print("Publisher and Subscriber Created")


    # start
    rate = rospy.Rate(50)
    last_request = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        print("time: {}".format(rospy.Time.now().to_sec() - last_request))
        pos_info = {"mav_pos": mav_pos, "mav_vel": mav_vel, "mav_R": mav_R}
        print("mav_pos: {}\nmav_vel: {}".format(mav_pos, mav_vel))

        if dj_action.dj == True:
            cmd = u.RotateAttackController(pos_info, pos_i, image_center)
            # 识别到图像才进行角速度控制
            if pos_i[1] > 0:
                command.orientation = cmd[0]
                command.thrust = cmd[1]
                print("cmd: {}".format(cmd))
            # # 否则hover
            else:
                command = AttitudeTarget()
        else:
            command = AttitudeTarget()
        print("command: {}".format([command.orientation, command.thrust]))
        local_vel_pub.publish(command)
        rate.sleep()
    rospy.spin()