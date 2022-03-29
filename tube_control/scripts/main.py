#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from swarm_msgs.msg import Pipeline, Pipeunit

class UAV:
    def __init__(self, drone_id):
        # parameters
        self.l = 5.0
        # status, with np.array
        self.drone_id = drone_id
        self.drone_name = "drone_{}".format(self.drone_id)
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_vel = np.array([0., 0., 0.])
        self.ksi = self.filtered(self.mav_pos, self.mav_vel)
        self.arrive = False
        # control command
        self.Vline = np.array([0.0, 0.0, 0.0])
        self.Vcollision = np.array([0.0, 0.0, 0.0])
        self.Vtube = np.array([0.0, 0.0, 0.0]) 
        self.Vheight = np.array([0.0, 0.0, 0.0])       
        self.tube_middle = []
        self.tube_left = []
        self.tube_right = []
        self.tube_middle_list = []
        self.tube_left_list = []
        self.tube_right_list = []
        # Subscribers and Publishers
        self.mav_pos_sub = rospy.Subscriber("{}/mavros/local_position/pose_cor".format(self.drone_name), PoseStamped, self.mav_pose_cb)
        self.mav_vel_sub = rospy.Subscriber("{}/mavros/local_position/velocity_local".format(self.drone_name), TwistStamped, self.mav_vel_cb)
        self.tube_sub = rospy.Subscriber("/pipeline/paths", Pipeline, self.tube_cb)
        self.vel_pub = rospy.Publisher('Pipeline_cmd', TwistStamped, queue_size=10)


    def filtered(self, pos, vel):
        return pos + vel / self.l

    def control(self, command):
        cmd_vel = TwistStamped()
        cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z = command[0], command[1], command[2]
        self.vel_pub.publish(cmd_vel)

    def update(self):
        self.ksi = self.filtered(self.mav_pos, self.mav_vel)

    def mav_pose_cb(self, msg):
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def mav_vel_cb(self, msg):
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
    
    def tube_cb(self, msg):
        self.tube_middle_list = []
        self.tube_left_list = []
        self.tube_right_list = []        
        for u in msg.units:
            self.tube_left_list.append([u.left.x, u.left.y])
            self.tube_right_list.append([u.right.x, u.right.y])
            self.tube_middle_list.append([u.middle.x, u.middle.y, u.middle.z])

        self.tube_left = np.array(self.tube_left_list)
        self.tube_right = np.array(self.tube_right_list)
        self.tube_middle = np.array(self.tube_middle_list)

class FreeFlight:
    def __init__(self, drone_num):
        # UAVs status
        self.drone_num = drone_num
        self.swarm = list()
        for i in range(self.drone_num):
            self.swarm.append(UAV(i+1))
        # parameters
        self.rs = 0.6
        self.ra = 2.0
        self.rt1 = 0.3
        self.rt2 = 0.6
        self.vm = 2.0
        self.k1 = 1.0
        self.k2 = 50.0
        self.k3 = 30.0
        self.kh = 1
        self.middle_locate = 1
        # ending flag 
        self.ending = False

    def controller(self):
        commands = list()
        # for each UAV
        for i in range(self.drone_num):
            # attractive potential

            self.middle_locate = 0

            dis_middle_min = 1e8
            for k in range(np.size(self.swarm[i].tube_middle,0)):
                relaive_middle = np.array([self.swarm[i].ksi[0],self.swarm[i].ksi[1]]) - np.array([self.swarm[i].tube_middle[k][0],self.swarm[i].tube_middle[k][1]])
                dis_middle = np.linalg.norm(relaive_middle)
                if dis_middle < dis_middle_min:
                    self.middle_locate = k
                    dis_middle_min = dis_middle

            rot = np.array([[0.0,-1.0],[1.0,0.0]])
            right_left = self.swarm[i].tube_right[self.middle_locate] - self.swarm[i].tube_left[self.middle_locate]
            C = np.matmul(rot,right_left.T)
            tc = C.T / np.linalg.norm(C)

            self.swarm[i].Vline[[0,1]] = self.sat(self.k1 * self.vm * tc, self.vm)

            # repulsive potentials

            self.swarm[i].Vcollision = np.array([0.0, 0.0, 0.0])
            for j in range(self.drone_num):
                if j == i: continue
                ksi_m = self.swarm[i].ksi - self.swarm[j].ksi
                nksi_m = np.linalg.norm(ksi_m)
                bij = self.k2 * self.dmysigma2(nksi_m,2*self.rs,self.rs+self.ra,0.0)
                self.swarm[i].Vcollision += bij * ksi_m / nksi_m

            mksii = 0.5 * (self.swarm[i].tube_right[self.middle_locate] + self.swarm[i].tube_left[self.middle_locate])
            rtksii = 0.5 * np.linalg.norm(right_left)
            dti = rtksii - np.linalg.norm(self.swarm[i].ksi[[0,1]] - mksii)
            ci = -1 * self.k3 * self.dmysigma2(dti,self.rt1,self.rt2,0.0) * (self.swarm[i].ksi[[0,1]] - mksii) / np.linalg.norm(self.swarm[i].ksi[[0,1]] - mksii)
            # Pt = np.array([[1.0,0.0],[0.0,1.0]]) - np.matmul(tc,tc.T)
            # self.swarm[i].Vtube[[0,1]] = np.matmul(Pt,ci)
            self.swarm[i].Vtube[[0,1]] = ci



            height_desire = self.swarm[i].tube_middle[self.middle_locate][2]
            self.swarm[i].Vheight[2] = self.kh * (height_desire-self.swarm[i].mav_pos[2])

            commands.append( (self.sat(self.swarm[i].Vline + self.swarm[i].Vcollision + self.swarm[i].Vtube + self.swarm[i].Vheight, self.vm)).tolist() )

            # if i == 2:
            #     print("Vtube:", self.swarm[i].Vtube)

        return commands

    def update(self):
        flag = True
        for i in range(self.drone_num):
            self.swarm[i].update()
            if self.swarm[i].arrive == False:
                flag = False
        self.ending = flag

    def sat(self, v, vm):
        normv = np.linalg.norm(v)
        if normv > vm:
            return vm / normv * v
        else:
            return v

    def dmysigma2(self, x, d1, d2, em):
        if x <= d1:
            return 1.0
        elif x <= d2 and x > d1:
            A = -(d2 * em - d1 * em + 2) / (d1 - d2) ** 3
            B = (-em * d1 **2 - em * d1 *d2 + 3 * d1 + 2 * em * d2 ** 2 + 3 * d2) / (d1 - d2) ** 3
            C = -(d2 * (-2 * em *d1 ** 2 + em * d1 * d2 + 6 * d1 + em * d2 ** 2)) / (d1 - d2) ** 3
            D = (d2 * (-em * d1 ** 2 * d2 + em * d1 * d2 ** 2 + 3 * d1 *d2 - d2 ** 2)) / ((d1 - d2) * (d1 ** 2 -2 * d1 * d2 + d2 ** 2))
            return A * x ** 3 + B * x ** 2 + C * x + D
        else:
            return 0.0

if __name__ == "__main__":
    rospy.init_node('tube_control', anonymous=True)
    drone_num = int(rospy.get_param("~drone_num"))
    drone_id = int(rospy.get_param("~drone_id"))
    drone_index = drone_id - 1

    # Destinations, custom
    ff = FreeFlight(drone_num)
    print("TubeControl Initialized")

    # main loop
    rate = rospy.Rate(50)
    while True:
        if ff.ending:
            break
        
        commands = ff.controller()
        ff.swarm[drone_index].control(commands[drone_index])
        ff.update()
        rate.sleep()
