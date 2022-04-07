#! /usr/bin/env python3
# coding=utf-8


from math import sqrt, atan2, sin, cos
import numpy as np
import rospy

# from simulation_environment import generate_gridmap, generate_targets, generate_uav_init
from simu_data_h import generate_map, generate_targets, generate_uav_init
from bresenham import bresenham_line

from std_msgs.msg import Int16
from decision_maker.msg import Action, Waypoint, Value, IdList, SwarmInfo, PlotPoint
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from swarm_msgs.msg import BoundingBoxes, BoundingBox


class other_part_simulation:
    EXPAND_NUM = 3
    def __init__(self, uav_num, swarm_num, level_id):
        self.uav_num = uav_num
        self.swarm_num = swarm_num
        self.level_id = level_id

        self.init_pos = generate_uav_init(self.level_id)
        self.finished_target = set()

        self.uav_speed = 2
        self.sight_range = 14
        self.resolution = 0.2
        # self.org_idx_in_global = [48.8, -19.6]
        self.org_idx_in_global = [35.8, -1.6]
        self.threshold_grid = 0.5
        if level_id == 1:
            self.std_height = 1.8
        elif level_id == 2:
            self.std_height = 6
        self.grid_map = generate_map(self.level_id)
        self.expand_map()
        self.random_tar = False
        if self.random_tar:
            _tar_pos = generate_targets(self.level_id)
            self.tar_pos = list(map(lambda c: self.grid2global_coor(c), _tar_pos))
        else:
            self.tar_pos = generate_targets(self.level_id)
        
        # self.tar_talker = rospy.Publisher('tar_plot', PlotPoint, queue_size=10)
        # self.uav_talker = rospy.Publisher('uav_plot', PlotPoint, queue_size=10)
        self.id_mavPublisher_dict = {key: value for (key, value) in [(uid, rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)) for uid in range(self.uav_num)]}
        self.id_mavPos_dict = {key: value for (key, value) in [(uid, None) for uid in range(self.uav_num)]}

        self.swarm_info_listener = rospy.Subscriber('swarm_info', SwarmInfo, self.swarm_info_update)
        self.swarm_info_dict = {}
        self.id_check_mult = 0.5
        self.new_uid_list = []
        self.interval = 0

        self.id_swarmId_dict = {}
        self.id_currentPos_dict = {key: value for (key, value) in [(uid, ipos) for uid, ipos in zip(list(range(self.uav_num)), self.init_pos)]}
        self.id_realposListener_dict = {key: value for (key, value) in [(uid, rospy.Subscriber("/drone_{}/mavros/local_position/pose_cor".format(uid+1), PoseStamped, self.update_uavpos, uid)) for uid in range(self.uav_num)]}

        # self.id_posSubscriber_dict = {key: value for (key, value) in [(uid, rospy.Subscriber('expect_pos' + str(uid), Waypoint, self.draw_pos, uid)) for uid in range(self.uav_num)]}
        # self.id_actionSubscriber_dict = {key: value for (key, value) in [(uid, rospy.Subscriber('expect_action' + str(uid), Action, self.draw_action, uid)) for uid in range(self.uav_num)]}

        # self.id_posPublisher_dict = {key: value for (key, value) in [(uid, rospy.Publisher('/drone_%s/mavros/local_position/pose_cor' % uid, PoseStamped, queue_size=10)) for uid in range(self.uav_num)]}
        # self.id_posTimer_dict = {key: value for (key, value) in [(uid, rospy.Timer(rospy.Duration(0.1), lambda x: self.send_simu_pos(x, uid))) for uid in range(self.uav_num)]}
        self.target_publisher = rospy.Publisher('/bounding_boxes' + str(self.level_id), BoundingBoxes, queue_size=10)
        # self.id_tarTimer_dict = {key: value for (key, value) in [(uid, rospy.Timer(rospy.Duration(0.2), lambda x: self.send_simu_tar(x, uid))) for uid in range(self.uav_num)]}
        self.map_publisher = rospy.Publisher('/map2d' + str(self.level_id), OccupancyGrid, queue_size=10)

    @staticmethod
    def distance_pos(pos1, pos2):
        return sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
    
    def swarm_info_update(self, msg):
        self.id_swarmId_dict[msg.data] = msg.swarm_id
        try:
            self.swarm_info_dict[msg.data] += 1
        except KeyError:
            self.swarm_info_dict[msg.data] = 1
            self.new_uid_list.append(msg.data)

    def swarm_info_check(self, msg=None):
        for uid in self.new_uid_list:
            self.id_currentPos_dict[uid] = generate_uav_init(self.level_id, uav_id=uid)
            # self.id_posSubscriber_dict[uid] = rospy.Subscriber('expect_pos' + str(uid), Waypoint, self.draw_pos, uid)
            # self.id_actionSubscriber_dict[uid] = rospy.Subscriber('expect_action' + str(uid), Action, self.draw_action, uid)
            self.id_mavPublisher_dict[uid] = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
            self.id_mavPos_dict[uid] = None
            self.id_realposListener_dict[uid] = rospy.Subscriber("/drone_{}/mavros/local_position/pose_cor".format(uid+1), PoseStamped, self.update_uavpos, uid)
        self.new_uid_list.clear()
        try:
            std_times = max(self.swarm_info_dict.values())
        except ValueError:
            std_times = 5
        clear_list = []
        for uid in self.swarm_info_dict:
            if self.swarm_info_dict[uid] < std_times * self.id_check_mult:
                clear_list.append(uid)
            else:
                self.swarm_info_dict[uid] = 0
        for uid in clear_list:
            self.swarm_info_dict.pop(uid)

    def update_uavpos(self, msg, *args):
        uid = args[0]
        self.id_currentPos_dict[uid] = (msg.pose.position.x, msg.pose.position.y)

    def draw_pos(self, msg, *args):
        uid = args[0]
        expect_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y) 
        # if self.distance_pos(self.id_currentPos_dict[uid], expect_pos) > self.uav_speed:
        #     theta = atan2(expect_pos[1] - self.id_currentPos_dict[uid][1], expect_pos[0] - self.id_currentPos_dict[uid][0])
        #     uav_pos = (self.id_currentPos_dict[uid][0] + self.uav_speed * cos(theta), self.id_currentPos_dict[uid][1] + self.uav_speed * sin(theta))
        # else:
        #     uav_pos = expect_pos
        
        mavpos = PoseStamped()
        mavpos.pose.position.x, mavpos.pose.position.y = expect_pos # uav_pos
        mavpos.pose.position.z = msg.pose.pose.position.z
        self.id_mavPos_dict[uid] = mavpos

        # self.id_mavPublisher_dict[uid].publish(mavpos)
        
        # uav_grid_idx = self.global2grid_coor(uav_pos)
        # last_grid_idx = self.global2grid_coor(self.id_currentPos_dict[uid])
        # uav_plot_point = PlotPoint()
        # uav_plot_point.id = uid
        # uav_plot_point.type = self.id_swarmId_dict[uid]
        # _point = Point()
        # _point.x, _point.y = uav_grid_idx
        # uav_plot_point.point = _point
        # _last_p = Point()
        # _last_p.x, _last_p.y = last_grid_idx
        # uav_plot_point.last_point = _last_p
        # self.uav_talker.publish(uav_plot_point)

        # self.id_currentPos_dict[uid] = uav_pos

    def draw_action(self, msg, *args):
        uid = args[0]
        uav_attack = msg.dj
        attack_tar = msg.id
        
        if uav_attack:
            attack_pos = self.tar_pos[attack_tar]
            uav_pos = self.id_currentPos_dict[uid]
            if self.distance_pos(attack_pos, uav_pos) < self.uav_speed:
                # tar_pos_idx = self.global2grid_coor(attack_pos)
                # tar_p = PlotPoint()
                # tar_p.id = attack_tar
                # tar_p.type = 1
                # _p = Point()
                # _p.x, _p.y = tar_pos_idx
                # tar_p.point = _p
                # self.tar_talker.publish(tar_p)
                pass  # send attack signal
                
                self.finished_target.add(attack_tar)

    # def send_simu_pos(self, msg, *args):
    #     uid = args[0]
    #     simu_pos = PoseStamped()
    #     simu_pos.pose.position.x, simu_pos.pose.position.y = self.id_currentPos_dict[uid]
    #     simu_pos.pose.position.z = 1.5
    #     self.id_posPublisher_dict[uid].publish(simu_pos)

    def send_simu_tar(self, msg, *args):
        uid = args[0]
        simu_tars = BoundingBoxes()
        for t_i in range(len(self.tar_pos)):
            if t_i in self.finished_target:
                continue
            t_pos = self.tar_pos[t_i]
            if self.distance_pos(t_pos, self.id_currentPos_dict[uid]) < self.sight_range:
                see_f = True
                # print(t_pos, self.global2grid_coor(t_pos), self.id_currentPos_dict[uid], self.global2grid_coor(self.id_currentPos_dict[uid]))
                line_pos = bresenham_line(self.global2grid_coor(t_pos), self.global2grid_coor(self.id_currentPos_dict[uid]))
                for _pos in line_pos:
                    if self.grid_map[_pos[0]][_pos[1]]:
                        see_f = False
                        break
                if not see_f:
                    continue
                simu_tar = BoundingBox()
                simu_tar.xmin = 0
                simu_tar.ymin = 0
                simu_tar.xmax = 10
                simu_tar.ymax = 10
                simu_tar.id = t_i
                s_t_p = Point()
                s_t_p.x, s_t_p.y = t_pos
                simu_tar.point = s_t_p
                simu_tars.bounding_boxes.append(simu_tar)
        self.target_publisher.publish(simu_tars)

    def send_simu_map(self, msg, *args):
        simu_occu = OccupancyGrid()
        simu_map = list(map(int, self.grid_map.flatten()))
        simu_occu.data = simu_map
        simu_occu.info.resolution = self.resolution
        simu_occu.info.origin.position.x, simu_occu.info.origin.position.y = self.org_idx_in_global
        simu_occu.info.origin.position.z = self.std_height
        simu_occu.info.height, simu_occu.info.width = self.grid_map.shape
        self.map_publisher.publish(simu_occu)
    
    def send_mavpos(self, uid):
        # _p = PoseStamped()
        # _p.pose.position.x, _p.pose.position.y, _p.pose.position.z = 0, 0, 1.8
        # self.id_mavPublisher_dict[uid].publish(_p)
        # return
        try:
            if self.id_mavPos_dict[uid]:
                self.id_mavPublisher_dict[uid].publish(self.id_mavPos_dict[uid])
        except KeyError:
            pass
        else:
            if not self.id_mavPos_dict[uid]:
                _p = PoseStamped()
                try:
                    # print(self.id_currentPos_dict[uid])
                    _p.pose.position.x, _p.pose.position.y = self.id_currentPos_dict[uid]
                    _p.pose.position.z = self.std_height
                except KeyError:
                    pass
                else:
                    self.id_mavPos_dict[uid] = _p
                    self.id_mavPublisher_dict[uid].publish(_p)


    # def initial_tar(self):
        # map_talker = rospy.Publisher('map_plot', OccupancyGrid, queue_size=10)
        # simu_occu = OccupancyGrid()
        # simu_map = list(map(int, self.grid_map.flatten()))
        # simu_occu.data = simu_map
        # simu_occu.info.height, simu_occu.info.width = self.grid_map.shape
        # map_talker.publish(simu_occu)

        # plt.matshow(self.grid_map, cmap=plt.cm.gray_r)
        # for tid in range(len(self.tar_pos)):
        #     tar = self.tar_pos[tid]
        #     d_pos = self.global2grid_coor(tar)
        #     plot_point = PlotPoint()
        #     plot_point.id = tid
        #     plot_point.type = 0
        #     _point = Point()
        #     _point.x, _point.y = d_pos
        #     plot_point.point = _point
        #     plot_point.type = 0
        #     self.tar_talker.publish(plot_point)

            # _h = plt.scatter(d_pos[1], d_pos[0], s=20, c=self.target_color[0])
            # self.tid_drawHandle_dict[tid] = _h
        # for uid in range(self.uav_num):
        #     u_pos = self.init_pos[uid]
        #     d_u_pos = self.global2grid_coor(u_pos)
        #     _h = plt.scatter(d_u_pos[1], d_u_pos[0], s=25, c='k', marker='x')
        #     self.uid_drawHandle_dict[uid] = _h
        # plt.ion()
        # plt.show()
        # plt.pause(0.01)

    def global2grid_coor(self, glb):
        return [int((glb[1] - self.org_idx_in_global[1]) / self.resolution), int((self.org_idx_in_global[0] - glb[0]) / self.resolution)]

    def grid2global_coor(self, grid):
        return [self.org_idx_in_global[0] - (grid[1] + 0.5) * self.resolution, (grid[0] + 0.5) * self.resolution + self.org_idx_in_global[1]]
    
    def expand_map(self):
        air_obs = set()
        for i in range(self.grid_map.shape[0]):
            for j in range(self.grid_map.shape[1]):
                if self.grid_map[i][j] > self.threshold_grid:
                    continue
                for di in range(-self.EXPAND_NUM, self.EXPAND_NUM + 1):
                    for dj in range(-self.EXPAND_NUM, self.EXPAND_NUM + 1):
                        try:
                            if self.grid_map[i + di][j + dj] > self.threshold_grid:
                                air_obs.add((i, j))
                                break
                        except IndexError:
                            continue
                    if (i, j) in air_obs:
                        break
        for pix in air_obs:
            self.grid_map[pix[0]][pix[1]] = 100

    def publish_main(self):
        self.interval += 1
        if self.interval % 5 == 0:
            self.swarm_info_check()
            # if self.interval <= 20:
            #     self.initial_tar()
        # print(list(self.swarm_info_dict.keys()))
        if self.interval % 10 == 0:
            self.send_simu_map(None, -1)
        for uid in list(self.swarm_info_dict.keys()):
            if uid not in self.id_currentPos_dict:
                continue
            # self.send_simu_pos(None, uid)
            # self.send_mavpos(uid)
            self.send_simu_tar(None, uid)


if __name__ == '__main__':
    rospy.init_node('simu_and_send', anonymous=True)
    uav_num = rospy.get_param("~uav_num")
    try:
        swarm_num = rospy.get_param("~swarm_num")
    except KeyError:
        swarm_num = 1
    try:
        level_id = rospy.get_param("~level_id")
    except KeyError:
        level_id = 1
    rate = rospy.Rate(10)
    simuer = other_part_simulation(uav_num, swarm_num, level_id)
    while not rospy.is_shutdown():
        simuer.publish_main()
        rate.sleep()

