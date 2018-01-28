#!/usr/bin/env python
# coding=utf-8

import numpy as np


import rospy
from swarm_msgs.msg import Pipeline,RflyObject

copterID_start = 10000

vehicleType,MotorRPMSMean = 152,0



class PiplineShower:
    def __init__(self):
        rospy.init_node('show_pipline_node', anonymous=True)
        self.mav_id = int(rospy.get_param("~mav_id"))-1
        self.param_x = rospy.get_param("~mav_x")
        self.param_y = rospy.get_param("~mav_y")
        self.param_z = rospy.get_param("~mav_z")
        self.copterID = copterID_start+1000*self.mav_id
        

        
        self.obj_pub = rospy.Publisher('rfly_object', RflyObject, queue_size=10)
        rospy.Subscriber("/pipeline/paths", Pipeline, self.tube_cb)
        rospy.spin()

    def tube_cb(self, msg):
        for _ in range(300):
            self.pubObject(vehicleType,[0]*3,[0]*3,[0]*3)

 
        last_left = msg.units[0].left
        last_right = msg.units[0].right
        
        
        self.resetID()
        for u in msg.units[1:]:
            PosE,AngEuler,Scale = self.cal_pos_size(last_left,u.left)
            self.pubObject(vehicleType,PosE,AngEuler,Scale)
            PosE,AngEuler,Scale = self.cal_pos_size(last_right,u.right)
            self.pubObject(vehicleType,PosE,AngEuler,Scale)
            last_left = u.left
            last_right = u.right
        self.resetID()


    def pubObject(self,vehicleType,PosE,AngEuler,Scale):
        '''
            mavros到UE4坐标系
            mavros          ue4
            pos_x          pos_y
            pos_y          pos_x
            pos_z          - pos_z
        '''
        self.copterID += 1
        mavros_x,mavros_y,mavros_z = PosE
        # PosE = [mavros_y+self.param_y, mavros_x+self.param_x, -mavros_z+self.param_z]
        PosE = [mavros_y+self.param_x, mavros_x+self.param_y, mavros_z+self.param_z]
        obj_msg = RflyObject()
        obj_msg.copterID = self.copterID
        obj_msg.vehicleType = vehicleType
        obj_msg.MotorRPMSMean = MotorRPMSMean
        obj_msg.PosE.x,obj_msg.PosE.y,obj_msg.PosE.z = PosE
        obj_msg.AngEuler.x,obj_msg.AngEuler.y,obj_msg.AngEuler.z = AngEuler
        obj_msg.Scale.x,obj_msg.Scale.y,obj_msg.Scale.z = Scale
        self.obj_pub.publish(obj_msg)

    def resetID(self):
        self.copterID = copterID_start
    
    # 计算方向和距离
    def cal_pos_size(self,start, end):
        PosE = [(start.x + end.x) / 2,(start.y + end.y) / 2,start.z]
        AngEuler = [0,0,np.pi/2-np.arctan2(end.y - start.y, end.x - start.x)]
        Scale = [np.linalg.norm([end.y - start.y, end.x - start.x]),0.05,1]
        return PosE,AngEuler,Scale



if __name__ == '__main__':
    s = PiplineShower()

