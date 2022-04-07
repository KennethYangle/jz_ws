#! /usr/bin/env python3
# coding=utf-8

# all published and subscribed coordinates in this file are in the global coordinate system(FLU)

import rospy
# import numpy as np
from numpy import array
from math import sqrt

from std_msgs.msg import Int16
# from geometry_msgs.msg import Point32, Pose
from decision_maker.msg import Action, Waypoint, Value, IdList, SwarmInfo
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from darknet_ros_msgs.msg import BoundingBoxes

from other_elements_h import Target, Environment
from center_uav import CenterObj
from path_plan_methods import LazyThetaStar
from bresenham import bresenham_line

X_BIAS, Y_BIAS = 0, 0

class UAVObj:
    def __init__(self, uav_id, center_flag, uav_size, other_vars):
        self.uav_id = uav_id
        self.center_flag = center_flag
        self.uav_size = uav_size  # 无人机外接圆半径
        self.swarm_id = other_vars['swarm_id']  # 无人机所在组的id
        try:
            self.safe_dis = other_vars['safe_distance']
        except KeyError:
            self.safe_dis = 2 * self.uav_size
        try:
            self.vision_dis = other_vars['vision_distance']
        except KeyError:
            self.vision_dis = 5 * self.uav_size
        try:
            self.uav_num = other_vars['uav_num']
        except KeyError:
            self.uav_num = 1
        # self.std_height = 1.8
        self.obs_threshold = 70
        self.no_tar_mult = 10  # 此数乘无人机数目得到一阈值，若目标未被发现的次数超过该阈值则认为完成该目标的任务
        self.step_time = 1

        self.position = [0, 0]
        self.uav_height = 0
        self.resource = 1
        self.receive_tar_num = 0
        self.uav_state = 0  # 无人机状态，0为进入仓库前，1为搜索状态(包括无目标时搜索和朝目标飞行的过程中)，2为打击状态

        ##############
        # self._temp_start = [13, -18]
        # self._temp_start = [0, 0]
        ##############

        self.next_position = self.position[::]
        self.path_pos_list = []
        self.path_msg = Path()
        # self.pos_msg = Waypoint()
        self.action_msg = Action()
        self.no_cur_tar_times = 0  # 当前目标在打击状态下未被检测到的次数，大于self.max_no_tar_times时认为该目标已被打击完成
        self.waiting = []
        self.done_list = []
        self.current_tar = None
        self.tmp_list = []
        self.tmp_tar_id = []
        self.est_list = []
        self.same_swarm_num = self.uav_num

        self.environment = Environment()
        self.path_planner = LazyThetaStar(self.environment.grid_map, obs_threshold=self.obs_threshold)

        self.target_listen = rospy.Subscriber(
            '/bounding_boxes' + str(self.swarm_id),
            BoundingBoxes,
            self.target_update_callback)

        self.uav_position_listen = rospy.Subscriber(
            "/drone_{}/mavros/local_position/pose_cor".format(int(self.uav_id)+1),
            PoseStamped,
            self.uav_pos_update_callback)

        self.map_listen = rospy.Subscriber(
            '/map2d' + str(self.swarm_id),
            OccupancyGrid,
            self.map_update_callback)

        self.assign_listen = rospy.Subscriber(
            'assign_result'+str(self.uav_id),
            IdList,  # target_id_list
            self.assigned_callback)

        self.finish_talk = rospy.Publisher('finish_signal' + str(self.swarm_id), Int16, queue_size=10)
        self.finish_listen = rospy.Subscriber(
            'finish_signal' + str(self.swarm_id),
            Int16,  # tar_id
            self.finish_callback)

        self.fitness_talk = rospy.Publisher('fitness' + str(self.swarm_id), Value, queue_size=10)
        # self.examine_timer = rospy.Timer(rospy.Duration(0.5), self.estimate_callback)

        self.pos_talk = rospy.Publisher('expect_pos'+str(self.uav_id), Path, queue_size=10)
        # self.pos_talk = rospy.Publisher('expect_pos'+str(self.uav_id), Waypoint, queue_size=10)
        # self.pos_timer = rospy.Timer(rospy.Duration(0.5), self.send_pos_callback)

        self.action_talk = rospy.Publisher('expect_action'+str(self.uav_id), Action, queue_size=10)
        # self.action_timer = rospy.Timer(rospy.Duration(0.5), self.send_action_callback)

        self.swarm_info_talker = rospy.Publisher('swarm_info', SwarmInfo, queue_size=10)
        self.center_info_listener = rospy.Subscriber('center_info', SwarmInfo, self.center_info_update)
        self.swarm_info_timer = rospy.Timer(rospy.Duration(0.05), self.send_id_callback)
        # self.center_check_timer = rospy.Timer(rospy.Duration(0.5), self.center_info_check)
        self.center_info_dict = {'c': 0, self.uav_id: 0}  # 无人机发送一次ID时self.uav_id项+1;收到中心返回ID时'c'项+1
        self.other_uid_dict = {}  # 接收中心返回的其他无人机的id，为了在损失中心后重新形成中心节点
        self.no_center_mult = 0.5  # 无人机收到中心返回的次数小于其发送次数的该倍数时认为中心结点缺失
        self.max_no_tar_times = self.no_tar_mult * self.uav_num

        self.program_timer = rospy.Timer(rospy.Duration(self.step_time), self.program_callback)

        if self.center_flag:
            self.center_obj = CenterObj(uav_num=self.same_swarm_num, swarm_id=self.swarm_id)
        else:
            self.center_obj = None

    @staticmethod
    def distance_pos(pos1, pos2):
        return sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def search_target(self):  #########################################
        if not hasattr(self, 'search_targets_list'):
            if self.swarm_id == 1:
                if self.uav_id % 2:
                    self.search_targets_list = [(25, 12)]
                else:
                    self.search_targets_list = [(10, 10), (25, 12)]
            elif self.swarm_id == 2:
                if self.uav_id % 2:
                    self.search_targets_list = [(5, 0), (25, 12)]
                else:
                    self.search_targets_list = [(24, 0), (25, 12)]
            else:
                self.search_targets_list = [(30, 12)]
        if len(self.environment.grid_map):
            uav_grid_pos = self.environment.real2grid(self.position)
            _search_pos = min(self.search_targets_list, key=lambda p: self.distance_pos(p, self.position))
            search_pos = (_search_pos[0] + X_BIAS, _search_pos[1] + Y_BIAS)
            tar_grid_pos = self.environment.real2grid(search_pos)
            print(uav_grid_pos, tar_grid_pos, 'pathplanpoints', self.environment.grid_map[tar_grid_pos[0]][tar_grid_pos[1]])
            self.path_pos_list = self.path_planner.path_plan(uav_grid_pos, tar_grid_pos)
            print(self.path_pos_list, 'pose_list')
            self.next_position = self.environment.grid2real(self.path_pos_list[-1])
            if self.distance_pos(self.position, search_pos) < 1 and len(self.search_targets_list) > 1:
                self.search_targets_list.remove(_search_pos)
            print(self.next_position, 'search')
            print('currenttarlist', [t.position for t in self.tmp_list])
        else:
            self.path_pos_list = [self.position]
            self.next_position = self.position

    def path_plan(self):
        if len(self.environment.grid_map):
            uav_grid_pos = self.environment.real2grid(self.position)
            tar_grid_pos = self.environment.real2grid(self.current_tar.position)
            self.path_pos_list = self.path_planner.path_plan(uav_grid_pos, tar_grid_pos)
            self.next_position = self.environment.grid2real(self.path_pos_list[-1])
            print(self.next_position, tar_grid_pos, 'pathplan')
        else:
            self.path_pos_list = [self.position]
            self.next_position = self.position

    def target_update_callback(self, msg):
        for tar in msg.bounding_boxes:
            tar_obj = Target(tar.id, (tar.point.x + X_BIAS, tar.point.y + Y_BIAS), (tar.xmax - tar.xmin) * (tar.ymax - tar.ymin))
            try:
                idx = self.tmp_list.index(tar_obj)
            except ValueError:
                self.tmp_list.append(tar_obj)
            else:
                self.tmp_list[idx].position = tar_obj.position
                self.tmp_list[idx].area = tar_obj.area

        if self.uav_state == 2:  # 检测当前任务是否完成，需要感知一直发boundin_boxes，即没有目标也发空列表
            no_cur_flag = True
            for tar in msg.bounding_boxes:
                if tar.id == self.current_tar.tar_id:
                    self.no_cur_tar_times = 0
                    no_cur_flag = False
                    break
            if no_cur_flag:
                self.no_cur_tar_times += 1
            if self.no_cur_tar_times > self.max_no_tar_times:  # 未检测到当前目标的次数超过最大次数
                self.no_cur_tar_times = 0
                cti = Int16()
                cti.data = self.current_tar.tar_id
                self.finish_talk.publish(cti)  # 发送当前目标打击完成的topic

    def uav_pos_update_callback(self, msg):
        self.position[::] = msg.pose.position.x, msg.pose.position.y
        self.uav_height = msg.pose.position.z
        if not self.uav_state:  # 当无人机为起飞状态(0)时
            if self.uav_height > 0.9 * self.environment.height:  # 起飞到高度大于0.9times 
                # self.uav_state = 1  # 将无人机设为搜索状态(1)
                if hasattr(self, '_temp_start') and self.distance_pos(self.position, self._temp_start) < 1:
                    self.uav_state = 1  # 将无人机设为搜索状态(1)
                    self.tmp_list.clear()
        
        # print('updatepos', self.position, self.height, self.uav_state)

    def map_update_callback(self, msg):
        if len(self.environment.grid_map):
            return
        self.environment.grid_map = array(msg.data).reshape((msg.info.height, msg.info.width))
        self.environment.grid_resolution = msg.info.resolution
        self.environment.org_in_global = (msg.info.origin.position.x + X_BIAS, msg.info.origin.position.y + Y_BIAS)
        self.environment.height = msg.info.origin.position.z
        self.path_planner.grid_map = self.environment.grid_map
        if not hasattr(self, '_temp_start'):
            if self.environment.height < 5:  ###########
                self._temp_start = [0, 0]
            else:
                self._temp_start = [13.5, 0]

    def assigned_callback(self, msg):
        for t in self.tmp_list:
            if t.tar_id in msg.target_id_list:
                if self.current_tar is None or t == self.current_tar:
                    self.waiting.insert(0, t)
                else:
                    if t not in self.waiting:
                        self.waiting.append(t)
            else:
                if t in self.waiting:
                    self.waiting.remove(t)
        try:
            self.current_tar = self.waiting.pop(0)
        except IndexError:
            self.current_tar = None
            self.uav_state = 1
        else:
            self.receive_tar_num += 1

    def finish_callback(self, msg):
        self.no_cur_tar_times = 0
        finish_tar = None
        for _tar in self.tmp_list:
            if _tar.tar_id == msg.data:
                finish_tar = _tar
                break
        try:
            self.tmp_list.remove(finish_tar)
            self.waiting.remove(finish_tar)
        except (ValueError, AttributeError):
            # rospy.loginfo('target has been cleared ' + str(self.uav_id))
            pass
        if self.current_tar and finish_tar and finish_tar == self.current_tar:
            if self.uav_state == 2:  
                self.done_list.append(self.current_tar)
            try:
                self.current_tar = self.waiting.pop(0)
            except IndexError:
                self.current_tar = None
            self.uav_state = 1

    def estimate_targets(self):  ##########################
        self.est_list.clear()
        self.tmp_tar_id.clear()
        for tar in self.tmp_list:
            distance = self.distance_pos(self.position, tar.position)
            done_busy = len(self.done_list)
            if self.current_tar is not None and self.current_tar.tar_id != tar.tar_id:
                done_busy += 5
            size_a = tar.area
            c_d, c_t, c_s = 1000, 2000, 1
            fitness = c_d * distance + c_t * done_busy - c_s * size_a
            self.est_list.append(fitness)
            self.tmp_tar_id.append(tar.tar_id)

    def estimate_callback(self, msg=None):
        self.estimate_targets()

        fit_talk = Value()
        fit_talk.uav_id = self.uav_id
        fit_talk.swarm_id = self.swarm_id
        for est, tid in zip(self.est_list, self.tmp_tar_id):
            fit_talk.fit_value_list.append(est)
            fit_talk.tar_id_list.append(tid)
        self.fitness_talk.publish(fit_talk)

    def takeoff_procedure(self):
        if not hasattr(self, '_temp_start'):
            return

        current_pos = PoseStamped()
        current_pos.pose.position.x, current_pos.pose.position.y = self.position[::]
        current_pos.pose.position.z = self.uav_height
        self.path_msg.poses.append(current_pos)
        
        pos_msg = PoseStamped()
        pos_msg.pose.position.x = self._temp_start[0]
        pos_msg.pose.position.y = self._temp_start[1]
        pos_msg.pose.position.z = self.environment.height
        self.path_msg.poses.append(pos_msg)
        self.path_msg.header.frame_id = "map"
        self.pos_talk.publish(self.path_msg)

    def send_pos_callback(self, msg=None):
        # if self.current_tar:
            # print('uid', self.uav_id, 'tid', self.current_tar.tar_id, 'state', self.uav_state)
        if not len(self.environment.grid_map):
            return
        if not self.tmp_list or not self.current_tar:
            self.search_target()
        # elif not self.current_tar:
        #     self.next_position = self.position[::]
        else:
            self.path_plan()
        for nxt_pos in self.path_pos_list:
            pos_msg = PoseStamped()
            nxt_p = self.environment.grid2real(nxt_pos)
            pos_msg.pose.position.x = nxt_p[0]
            pos_msg.pose.position.y = nxt_p[1]
            pos_msg.pose.position.z = self.environment.height
            self.path_msg.poses.append(pos_msg)
        self.path_msg.header.frame_id = "map"
        current_pos = PoseStamped()
        current_pos.pose.position.x, current_pos.pose.position.y = self.position[::]
        current_pos.pose.position.z = self.uav_height
        self.path_msg.poses.append(current_pos)
        self.path_msg.poses.reverse()
        self.pos_talk.publish(self.path_msg)

    def send_action_callback(self, msg=None):
        if not len(self.environment.grid_map):
            return
        print(self.current_tar.position if self.current_tar else None, 'currenttar')
        # if self.current_tar:
        #     print(self.distance_pos(self.position, self.current_tar.position))
        if self.current_tar and self.distance_pos(self.position, self.current_tar.position) < self.vision_dis:
            self.action_msg.dj = True
            self.uav_state = 2
            line_pos = bresenham_line(self.environment.real2grid(self.position), self.environment.real2grid(self.current_tar.position))
            for pos in line_pos:
                if self.environment.grid_map[pos[0], pos[1]] > self.obs_threshold:
                    self.action_msg.dj = False
                    self.uav_state = 1
                    break
        else:
            self.action_msg.dj = False
            self.uav_state = 1
        try:
            self.action_msg.id = self.current_tar.tar_id
        except AttributeError:
            self.action_msg.id = -1
        self.action_talk.publish(self.action_msg)

    def send_id_callback(self, msg):
        id_msg = SwarmInfo()
        id_msg.data = self.uav_id
        id_msg.swarm_id = self.swarm_id
        self.swarm_info_talker.publish(id_msg)
        self.center_info_dict[self.uav_id] += 1

    def center_info_update(self, msg):
        if msg.swarm_id != self.swarm_id:
            return
        if msg.data == self.uav_id:
            self.center_info_dict['c'] += 1
        else:
            try:
                self.other_uid_dict[msg.data] += 1
            except KeyError:
                self.other_uid_dict[msg.data] = 1

    def center_info_check(self, msg=None):
        if self.center_info_dict['c'] < self.center_info_dict[self.uav_id] * self.no_center_mult:
            change_flag = True
            for uid in list(self.other_uid_dict.keys()):
                if self.other_uid_dict[uid] >= self.center_info_dict['c'] * self.no_center_mult:
                    change_flag = False if uid < self.uav_id else True
            if change_flag:
                self.center_flag = 1
                self.center_obj = CenterObj(uav_num=self.same_swarm_num, swarm_id=self.swarm_id)
        elif self.center_info_dict['c'] > self.center_info_dict[self.uav_id]:
            # 此函数最后会清空旧的other_uid_dict，若恰巧这时损失了中心，则无人机无法正确的选择新中心且将自己作为中心，即可能会出现多个中心，在此处理
            change_flag = False
            for uid in self.other_uid_dict:
                if self.other_uid_dict[uid] >= self.center_info_dict[self.uav_id] * self.no_center_mult:
                    change_flag = True if uid < self.uav_id else False
            if change_flag:
                self.center_flag = 0
                self.center_obj = None

        self.center_info_dict['c'] = 0
        self.center_info_dict[self.uav_id] = 0
        self.other_uid_dict.clear()

    def program_callback(self, msg=None):
        self.path_msg.poses.clear()
        self.center_info_check()
        if not self.uav_state:
            self.takeoff_procedure()
            return
        self.estimate_callback()
        self.send_action_callback()
        self.send_pos_callback()


def main():
    info_dict = {}
    u_id = rospy.get_param("~uav_id")
    c_fl = rospy.get_param("~cen_flag")
    u_sz = rospy.get_param("~uav_size")
    try:
        info_dict['swarm_id'] = rospy.get_param("~swarm_id")
    except KeyError:
        info_dict['swarm_id'] = -1
    try:
        info_dict['safe_distance'] = rospy.get_param("~safe_distance")
    except KeyError:
        pass
    try:
        info_dict['vision_distance'] = rospy.get_param("~vision_distance")
    except KeyError:
        pass
    try:
        info_dict['uav_num'] = rospy.get_param("~uav_num")
    except KeyError:
        pass

    uav_obj = UAVObj(uav_id=u_id, center_flag=c_fl, uav_size=u_sz, other_vars=info_dict)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('uav_node', anonymous=True)
    try:
        main()
    except rospy.ROSInterruptException:
        pass

