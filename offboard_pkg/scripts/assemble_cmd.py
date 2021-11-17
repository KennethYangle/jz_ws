#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from swarm_msgs.msg import Action


class Assemble:

    def __init__(self, param_id):
        self.start_time = time.time()
        
        self.pipeline_cmd = TwistStamped()
        self.dj_cmd = TwistStamped()
        self.obs_cmd = TwistStamped()
        self.dj_action = Action()

        self.Pipeline_cmd_sub = rospy.Subscriber('Pipeline_cmd', TwistStamped, self.Pipeline_cmd_callback)
        self.DJ_cmd_sub = rospy.Subscriber('DJ_cmd', TwistStamped, self.DJ_cmd_callback)
        self.Obs_cmd_sub = rospy.Subscriber('Obs_cmd', TwistStamped, self.Obs_cmd_callback)
        self.Expect_action_sub = rospy.Subscriber('expect_action'+str(param_id), Action, self.Expect_action_callback)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    def Pipeline_cmd_callback(self, msg):
        self.pipeline_cmd = msg

    def DJ_cmd_callback(self, msg):
        self.dj_cmd = msg

    def Obs_cmd_callback(self, msg):
        self.obs_cmd = msg

    def Expect_action_callback(self, msg):
        self.dj_action = msg

        
    def test_pos_ENU_toland(self):
        print("=========================")
        print("Taking off...")
        self.px.takeoff_pos(h=1.5)

        print("=========================")

        # print("Run 5s to land")
        # self.px.moveByVelocityYawrateENU(N=0.5)
        # time.sleep(5)
        # self.px.moveByVelocityYawrateENU(E=-0.5)
        # time.sleep(5)
        self.px.moveByPosENU(-4.74, 19, 1.8)
        time.sleep(15)
        # self.px.moveByPosENU(0,0)
        # time.sleep(15)

        print("=========================")
        print("Landing...")
        # self.px.land()

    def begin_task(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.dj_action.dj == True:
                self.vel_pub.publish(self.dj_cmd)
            else:
                self.vel_pub.publish(self.pipeline_cmd)
            rate.sleep()

if __name__=="__main__":
    rospy.init_node("assemble", anonymous=True)
    param_id = rospy.get_param("~drone_id")

    ass = Assemble(param_id)
    ass.begin_task()