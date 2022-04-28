#!/usr/bin/env python
# coding=utf-8

import numpy as np
import rospy
from swarm_msgs.msg import Pipeline,RflyObject

copterID = 10000
vehicleType = 152   # red ball
MotorRPMSMean = 0
# target_origin = [25,0,-2]
# omega = 0.2          # T = 2*pi/omega
target_origin = [60,0,-1.7]
omega = 0.1        # T = 2*pi/omega
A = 5
B = 10


class TargetUAV:
    def __init__(self):
        rospy.init_node('target_uav_node', anonymous=True)
        self.mav_id = int(rospy.get_param("~mav_id"))-1
        self.param_x = rospy.get_param("~mav_x")
        self.param_y = rospy.get_param("~mav_y")
        self.param_z = rospy.get_param("~mav_z")
        self.obj_pub = rospy.Publisher('rfly_object', RflyObject, queue_size=10)

    def pubObject(self,vehicleType,PosE,AngEuler=[0,0,0],Scale=[1,1,1]):
        mavros_x,mavros_y,mavros_z = PosE
        PosE = [mavros_x+self.param_x, mavros_y+self.param_y, mavros_z+self.param_z]
        obj_msg = RflyObject()
        obj_msg.copterID = copterID
        obj_msg.vehicleType = vehicleType
        obj_msg.MotorRPMSMean = MotorRPMSMean
        obj_msg.PosE.x,obj_msg.PosE.y,obj_msg.PosE.z = PosE
        obj_msg.AngEuler.x,obj_msg.AngEuler.y,obj_msg.AngEuler.z = AngEuler
        obj_msg.Scale.x,obj_msg.Scale.y,obj_msg.Scale.z = Scale
        self.obj_pub.publish(obj_msg)

    def start(self):
        # k = 0
        k = np.random.randint(1000)
        time_interval = 0.02
        rate = rospy.Rate(1/time_interval)
        while not rospy.is_shutdown():
            x = A*np.sin(2*omega*k*time_interval) + target_origin[0]
            y = B*np.sin(omega*k*time_interval)   + target_origin[1]
            z = target_origin[2]
            self.pubObject(vehicleType, [x,y,z], Scale=[0.2,0.2,0.2])
            k += 1
            rate.sleep()


if __name__ == '__main__':
    s = TargetUAV()
    s.start()
