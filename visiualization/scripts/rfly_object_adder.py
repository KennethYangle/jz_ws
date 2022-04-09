#!/usr/bin/env python
# coding=utf-8

import socket
import struct

import rospy
from swarm_msgs.msg import RflyObject


class RflyObjectAdder:
    def __init__(self):
        rospy.init_node('rfly_object_adder', anonymous=True)
        self.mav_id = int(rospy.get_param("~mav_id"))-1
        self.param_ip = rospy.get_param("~ue4_ip")
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber("/rfly_object", RflyObject, self.rfly_object_cb)
        rospy.spin()

    def rfly_object_cb(self, msg):
        copterID = msg.copterID
        vehicleType = msg.vehicleType

        MotorRPMSMean = msg.MotorRPMSMean
        PosE = [msg.PosE.x,msg.PosE.y,msg.PosE.z]
        AngEuler = [msg.AngEuler.x,msg.AngEuler.y,msg.AngEuler.z]
        Scale = [msg.Scale.x,msg.Scale.y,msg.Scale.z]
        self.send2UE4(copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,Scale)
    
    # 世界系下面的坐标 
    def send2UE4(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,Scale):
        buf = struct.pack( "3i10f",1234567890, copterID,vehicleType,MotorRPMSMean, PosE[0],PosE[1],PosE[2], AngEuler[0],AngEuler[1],AngEuler[2], Scale[0],Scale[1],Scale[2])
        self.udp_socket.sendto(buf, (self.param_ip, 20010+self.mav_id))

if __name__ == '__main__':
    s = RflyObjectAdder()

