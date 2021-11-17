#!/usr/bin/env python
#coding=utf-8

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped

class UAV:
    def __init__(self, drone_id, waypoint):
        # parameters
        self.l = 5.0
        # status, with np.array
        self.drone_id = drone_id
        self.drone_name = "drone_{}".format(self.drone_id)
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_vel = np.array([0., 0., 0.])
        self.waypoint = np.array(waypoint)
        self.ksi = self.filtered(self.mav_pos, self.mav_vel)
        self.arrive = False
        # control command
        self.attraction = np.array([0.0, 0.0, 0.0])
        self.repulsion = np.array([0.0, 0.0, 0.0])
        # Subscribers and Publishers
        self.mav_pos_sub = rospy.Subscriber("{}/mavros/local_position/pose_cor".format(self.drone_name), PoseStamped, self.mav_pose_cb)
        self.mav_vel_sub = rospy.Subscriber("{}/mavros/local_position/velocity_local".format(self.drone_name), TwistStamped, self.mav_vel_cb)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    def filtered(self, pos, vel):
        return pos + vel / self.l

    def control(self, command):
        cmd_vel = TwistStamped()
        cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.linear.z = command[0], command[1], command[2]
        self.vel_pub.publish(cmd_vel)

    def update(self):
        self.ksi = self.filtered(self.mav_pos, self.mav_vel)
        if np.linalg.norm(self.mav_pos - self.waypoint) <= 0.1:
            self.arrive = True
        else:
            self.arrive = False

    def mav_pose_cb(self, msg):
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        # q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        # self.mav_yaw = np.arctan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
        # self.mav_R = np.array([[q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
        #                 [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        #                 [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]])

    def mav_vel_cb(self, msg):
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])



class FreeFlight:
    def __init__(self, drone_num, waypoints):
        # UAVs status
        self.drone_num = drone_num
        self.swarm = list()
        for i in range(self.drone_num):
            self.swarm.append(UAV(i+1, waypoints[i]))
        # parameters
        self.rs = 0.6
        self.ra = 2.0
        self.vm = 2.0
        self.k1 = 1.0
        self.k2 = 1.0
        self.eps = 1e-6
        self.eps_s = 1e-6
        # ending flag 
        self.ending = False

    def controller(self):
        commands = list()
        # for each UAV
        for i in range(self.drone_num):
            # attractive potential
            ksi_wp = self.swarm[i].ksi - self.swarm[i].waypoint
            self.swarm[i].attraction = -1 * self.sat(self.k1 * ksi_wp, self.vm)

            # repulsive potentials
            self.swarm[i].repulsion = np.array([0.0, 0.0, 0.0])
            for j in range(self.drone_num):
                if j == i: continue
                ksi_m = self.swarm[i].ksi - self.swarm[j].ksi
                nksi_m = np.linalg.norm(ksi_m)
                den = (1+self.eps)*nksi_m - 2*self.rs*self.s(nksi_m/2/self.rs, self.eps_s)
                num = self.dsigma_m(nksi_m)*den - self.sigma_m(nksi_m)*( 1+self.eps-2*self.rs*self.ds(nksi_m/2/self.rs, self.eps_s) )
                b = self.k2 / nksi_m * num / den**2
                self.swarm[i].repulsion += -1 * b * ksi_m

            commands.append( (self.sat(self.swarm[i].attraction + self.swarm[i].repulsion, self.vm)).tolist() )
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

    def s(self, x, e):
        x2 = 1 + e / np.tan( 67.5/180*np.pi )
        x1 = x2 - np.sin( 45/180*np.pi ) * e
        if x <= x1:
            return x
        elif x <= x2:
            return 1 - e + np.sqrt( e**2 - (x-x2)**2 )
        else:
            return 1.0

    def ds(self, x, e):
        x2 = 1 + e / np.tan( 67.5/180*np.pi )
        x1 = x2 - np.sin( 45/180*np.pi ) * e
        if x <= x1:
            return 1.0
        elif x <= x2:
            return (x2 - x) / np.sqrt( e**2 - (x-x2)**2 )
        else:
            return 0.0

    def sigma_m(self, x):
        d1 = 2 * self.rs
        d2 = self.ra + self.rs
        if x <= d1:
            return 1.0
        elif x <= d2:
            A = -2 / (d1 - d2) ** 3
            B = 3 * (d1 + d2) / (d1 - d2) ** 3
            C = -6 * d1 * d2 / (d1 - d2) ** 3
            D = d2**2 * (3*d1 - d2) / (d1 - d2) ** 3
            return A * x**3 + B * x**2 + C * x + D
        else:
            return 0.0

    def dsigma_m(self, x):
        d1 = 2 * self.rs
        d2 = self.ra + self.rs
        if x <= d1:
            return 0.0
        elif x <= d2:
            A = -2 / (d1 - d2) ** 3
            B = 3 * (d1 + d2) / (d1 - d2) ** 3
            C = -6 * d1 * d2 / (d1 - d2) ** 3
            return 3*A * x**2 + 2*B * x + C
        else:
            return 0.0


if __name__ == "__main__":
    rospy.init_node('freeflight', anonymous=True)
    drone_num = int(rospy.get_param("~drone_num"))
    drone_id = int(rospy.get_param("~drone_id"))
    drone_index = drone_id - 1

    # Destinations, custom
    waypoints = [[6,6,2], [0,6,2], [6,0,2]]
    ff = FreeFlight(drone_num, waypoints)
    print("FreeFlight Initialized")

    # main loop
    rate = rospy.Rate(50)
    while True:
        if ff.ending:
            break
        
        commands = ff.controller()
        ff.swarm[drone_index].control(commands[drone_index])
        ff.update()
        rate.sleep()
