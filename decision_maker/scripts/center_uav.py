#! /usr/bin/env python3
# coding=utf-8


import rospy
from numpy import array, insert
from scipy.optimize import linear_sum_assignment
from modified_hungarian import unbalanced_assignment

from std_msgs.msg import Int16
from decision_maker.msg import Value, IdList, SwarmInfo


class CenterObj:
    def __init__(self, uav_num, swarm_id=-1):
        self.uav_num = uav_num
        self.swarm_id = swarm_id

        self.uav_chg_flag = False  # 当集群内的无人机发生变化时转为True
        self.tar_chg_flag = False  # 当目标发生变化时转为True

        self.assigned_talk_dict = {}  # {uid: rospy.Publisher(uid, ...)} 中心对每个无人机的Publisher对象
        self.assigned_result_dict = {}  # {uid: IdList()} 中心传回无人机的指派结果
        self.uid_set = set()  # 指派算法开始时中心接收到了这些无人机发来的(对目标的)评估值，指派完成后clear()
        self.tid_set = set()  # 指派算法开始时中心接收到了这些目标的评估值，指派完成后clear()
        self.uid_tnum_dict = {}  # {uid: tar_num} 在上次运行指派算法时无人机uid发来中心的目标数量
        self.uid_tfd_dict = {}  # {uid: {tid: fit}} 无人机uid发来中心的关于目标tid的评估值fit

        self.swarm_info_dict = {}  # {uid: send_times} 无人机发来的其uid的次数，用于判断集群内有哪些无人机
        self.id_check_mult = 0.5  # 某无人机发送其ID的次数少于最大次数的该倍数时认为离开本群体
        self.center_info_talker = rospy.Publisher('center_info', SwarmInfo, queue_size=10)
        self.swarm_info_listener = rospy.Subscriber('swarm_info', SwarmInfo, self.swarm_info_update)
        # self.swarm_check_timer = rospy.Timer(rospy.Duration(0.5), self.swarm_info_check)

        fit_str = 'fitness' + str(self.swarm_id) if self.swarm_id >= 0 else 'fitness'
        self.fitness_listen = rospy.Subscriber(
            fit_str,
            Value,
            self.fitness_callback)

        self.assign_timer = rospy.Timer(rospy.Duration(0.5), self.assign_callback)

    def swarm_info_update(self, msg):
        if msg.swarm_id != self.swarm_id:
            return
        try:
            self.swarm_info_dict[msg.data] += 1
        except KeyError:
            self.swarm_info_dict[msg.data] = 1
            self.uav_chg_flag = True
        cen_msg = SwarmInfo()
        cen_msg.data = msg.data
        cen_msg.swarm_id = self.swarm_id
        self.center_info_talker.publish(cen_msg)

    def swarm_info_check(self, msg=None):
        if not self.swarm_info_dict:
            return
        std_times = max(self.swarm_info_dict.values())
        clear_list = []
        for uid in self.swarm_info_dict:
            if self.swarm_info_dict[uid] < std_times * self.id_check_mult:
                clear_list.append(uid)
            else:
                self.swarm_info_dict[uid] = 0
        for uid in clear_list:
            self.swarm_info_dict.pop(uid)
            try:
                self.uid_set.remove(uid)
            except KeyError:
                pass
        if self.uav_num != len(self.swarm_info_dict):
            self.uav_chg_flag = True
            self.uav_num = len(self.swarm_info_dict)

    def fitness_callback(self, msg):
        if msg.swarm_id != self.swarm_id:
            return
        if msg.uav_id not in self.assigned_talk_dict:
            assigned_talk = rospy.Publisher(
                'assign_result'+str(msg.uav_id),
                IdList,  # target_id_list
                queue_size=10)
            self.assigned_talk_dict[msg.uav_id] = assigned_talk
            self.assigned_result_dict[msg.uav_id] = IdList()
        if msg.fit_value_list:  # 这个判断意味着当有目标被搜索到时，在当前群内的无人机都需要发送非空的评估值表，这样才算接收到了该无人机的评估值
            self.uid_set.add(msg.uav_id)
        # 检查目标是否有变化分为两步，第一步检查是否有新目标
        for tid, fit in zip(msg.tar_id_list, msg.fit_value_list):
            try:
                if tid not in self.uid_tfd_dict[msg.uav_id]:
                    self.tar_chg_flag = True
                self.uid_tfd_dict[msg.uav_id][tid] = fit
            except KeyError:
                self.uid_tfd_dict[msg.uav_id] = {tid: fit}
                self.uav_chg_flag = True
            self.tid_set.add(tid)
        # 第二步检查无人机传给中心的目标数有无变化
        try:
            if self.uid_tnum_dict[msg.uav_id] != len(msg.tar_id_list):
                self.tar_chg_flag = True
        except KeyError:
                self.uav_chg_flag = True
        finally:
            self.uid_tnum_dict[msg.uav_id] = len(msg.tar_id_list)

    def assign_callback(self, msg):
        self.swarm_info_check()
        # print('uavnum', self.uav_num)
        # print(0, self.uav_chg_flag, self.tar_chg_flag, len(self.uid_set), self.uav_num, self.swarm_info_dict.keys(), self.uid_set, self.tid_set)
        if (self.uav_chg_flag or self.tar_chg_flag) and (len(self.uid_set) == self.uav_num and self.swarm_info_dict.keys() == self.uid_set):  # 判断无人机与目标是否变化and群内无人机是否都将评估值传到中心
            # print(1, self.uav_chg_flag, self.tar_chg_flag, len(self.uid_set), self.uav_num, self.swarm_info_dict.keys(), self.uid_set, self.tid_set)
            # 当某个无人机发现一个新目标时，其他无人机还没收到新目标的消息，但运行了评估函数并publish了Value，这时会导致无人机的目标数不一致，在这进行处理
            max_fit = float('-inf')
            add_tuple_list = []
            for uid in self.uid_set:
                for tid in self.tid_set:
                    if tid not in self.uid_tfd_dict[uid]:
                        add_tuple_list.append((uid, tid))
                    else:
                        max_fit = self.uid_tfd_dict[uid][tid] if self.uid_tfd_dict[uid][tid] > max_fit else max_fit
            for _tup in add_tuple_list:
                self.uid_tfd_dict[_tup[0]][_tup[1]] = max_fit
            # 重置重分配条件标志位
            self.uav_chg_flag = False
            self.tar_chg_flag = False
            for u_i in self.assigned_result_dict:
                self.assigned_result_dict[u_i].target_id_list.clear()
            # 计算每个目标需要几架无人机
            try:
                assign_num = self.uav_num // len(self.tid_set)
            except ZeroDivisionError:
                assign_num = self.uav_num
            if assign_num < 1:
                assign_num = 1
            elif assign_num > self.uav_num // 2 and self.uav_num > 1:
                assign_num = self.uav_num // 2
            if assign_num < 2 and self.uav_num > 2:
                assign_num = 2
            # 生成价值矩阵
            tid_list = list(self.tid_set)
            uid_list = list(self.uid_set)
            assign_matrix = array([[self.uid_tfd_dict[uid][tid] for tid in tid_list] for uid in uid_list])
            # 任务分配算法
            # print(len(self.tid_set), self.uav_num, assign_num, 'assignnum')
            for _ in range(assign_num):
                if len(tid_list) <= len(uid_list):
                    assign_uav, assign_tar = linear_sum_assignment(assign_matrix)
                else:
                    assign_uav, assign_tar = unbalanced_assignment(assign_matrix)
                for ui, ti in zip(assign_uav, assign_tar):
                    self.assigned_result_dict[uid_list[ui]].target_id_list.append(tid_list[ti])
                    assign_matrix[ui][ti] += max_fit
            # for t_i in tid_list:
            #     uid_list.sort(key=lambda x: self.uid_tfd_dict[x][t_i])
            #     for r_i in range(assign_num):
            #         u_i = uid_list[r_i]
            #         self.assigned_result_dict[u_i].target_id_list.append(t_i)
            for a_talk, a_result in zip(self.assigned_talk_dict.values(), self.assigned_result_dict.values()):
                a_talk.publish(a_result)
            self.uid_set.clear()
            self.tid_set.clear()


def main():
    center_obj = CenterObj(uav_num=3)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

