#!/usr/bin/env python
# coding=utf-8

import time
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from swarm_msgs.msg import Action

def spin():
    rospy.spin()

class Assemble:

    def __init__(self, px):
        self.start_time = time.time()
        self.px = px
        
        self.pipeline_cmd = TwistStamped()
        self.dj_cmd = TwistStamped()
        self.obs_cmd = TwistStamped()
        self.dj_action = Action()

        self.Pipeline_cmd_sub = rospy.Subscriber('Pipeline_cmd', TwistStamped, self.Pipeline_cmd_callback)
        self.DJ_cmd_sub = rospy.Subscriber('DJ_cmd', TwistStamped, self.DJ_cmd_callback)
        self.Obs_cmd_sub = rospy.Subscriber('Obs_cmd', TwistStamped, self.Obs_cmd_callback)
        self.Expect_action_sub = rospy.Subscriber('expect_action'+str(param_id), Action, self.Expect_action_callback)

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
        print("=========================")
        print("Taking off...")

        self.px.takeoff_pos(h=1.5)

        for circle_id in range(len(self.setpoints)):
            # time.sleep(3)
            print("=========================")
            print("Now try to pass circle {}...".format(circle_id+1))
            self.task_cross_circle(circle_id, mode="minsnap")
            # self.task_cross_circle(circle_id, mode="moveToPosition")
            # if self.is_calibration:
            #     self.calibration_mavpos(circle_id)

        print("=========================")
        print("Planning to land point...")
        self.px.planning(self.setpoint_land[0]+self.px.mav_pos0[0], self.setpoint_land[1]+self.px.mav_pos0[1], self.setpoint_land[2]+self.px.mav_pos0[2])
        self.px.idle()
        while self.px.trigger_land != 1:
            time.sleep(0.02)

        print("=========================")
        print("Landing...")
        # self.moveToPosition(
        # self.setpoint_land[0], self.setpoint_land[1], self.setpoint_land[2])


if __name__=="__main__":
    rospy.init_node('assemble', anonymous=True)
    spin_thread = threading.Thread(target=spin)
    spin_thread.start()
    param_id = rospy.get_param("~drone_id")
