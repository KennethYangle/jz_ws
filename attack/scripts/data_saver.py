#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from swarm_msgs.msg import RflyObject
import time


class DataSaver:
    def __init__(self,start_time=time.time()):


        self.local_pose_vec = None
        self.obj_pose_vec = None

        self.start_time = start_time
        self.last_update_time = start_time
        # ENU系的位置
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        rospy.Subscriber('rfly_object', RflyObject, self.rfly_obj_callback)
        rospy.init_node("save_node",anonymous=True,disable_signals=True)
        self.save_path = param_id = rospy.get_param("~save_path")

    def start(self):
        file_name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        f = open("{}/{}.txt".format(self.save_path,file_name),"w")
        # f.close()


        save_time = time.time()
        data_txt = "{:.3f},"+("{:.6f},"*9)[:-1]+"\n"
        while not rospy.is_shutdown():
            if time.time()-save_time > 0.05 and (self.local_pose_vec is not None) and (self.obj_pose_vec is not None):
                save_time = time.time()
                pos_vec = self.local_pose_vec
                obj_vec = self.obj_pose_vec
                vel_vec = self.local_vel_vec
                # f = open("{}.txt".format(file_name),"a")
                f.write(
                    data_txt.format(time.time(),
                        vel_vec[0],vel_vec[1],vel_vec[2],
                        pos_vec[0],pos_vec[1],pos_vec[2],
                        obj_vec[0],obj_vec[1],obj_vec[2],
                    )
                )
                # f.close()
                # print(
                #     data_txt.format(time.time(),
                #         pos_vec[0],pos_vec[1],pos_vec[2],
                #         obj_vec[0],obj_vec[1],obj_vec[2],
                #     )
                # )
            if (self.last_update_time != self.start_time) and (time.time()- self.last_update_time > 5):
                break
            time.sleep(0.01)
        f.close()
        print("data save done")
    
    def local_pose_callback(self, msg):
        pos = msg.pose.position
        pose_vec = [pos.x,pos.y,pos.z]
        self.local_pose_vec = pose_vec
        self.last_update_time = time.time()

    # ENU的速度
    def local_vel_callback(self, msg):
        vel = msg.twist.linear
        vel_vec = [vel.x,vel.y,vel.z]
        self.local_vel_vec = vel_vec
        self.last_update_time = time.time()
    
    # ENU的速度
    def rfly_obj_callback(self, msg):
        self.obj_pose_vec = [msg.PosE.x,msg.PosE.y,msg.PosE.z]
        self.last_update_time = time.time()




if __name__ == '__main__':
    saver = DataSaver()
    saver.start()




