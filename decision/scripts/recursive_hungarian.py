#!/usr/bin/env python
# coding=utf-8

import numpy as np
from scipy.optimize import linear_sum_assignment


# Recursive Hungarian Algorithm for Dynamic Environments
# 输入：无人机位置、目标位置，均为二维numpy
class RHA2:
    def __init__(self, agent_pos, task_pos):
        self.agent_pos = agent_pos
        self.task_pos = task_pos

        col = []
        for i in range(len(self.task_pos)):
            if all(self.task_pos[i] == np.array([-1, -1, -1])):
                col.append(i)
        self.task_pos = np.delete(self.task_pos, col, axis=0)

        self.m = len(self.agent_pos)
        self.n = len(self.task_pos)

    def deal(self):
        # 初始化
        max_mn = max(self.m, self.n)
        cost = np.zeros([max_mn, max_mn])
        for i in range(self.m):
            for j in range(self.n):
                cost[i][j] = np.linalg.norm(self.agent_pos[i]-self.task_pos[j])
        uavs_pos_record = []
        uavs_pos_record.append(self.agent_pos)

        # 开始
        if self.m < self.n:
            while self.m < self.n:
                # 添加虚拟agent，补齐cost矩阵
                for i in range(self.m, self.n):
                    for j in range(self.n):
                        cost[i][j] = 0x3f3f3f3f

                row_ind, col_ind = linear_sum_assignment(cost)
                self.agent_pos = self.task_pos[col_ind[:self.m]]   # 更新agent位置
                self.task_pos = np.delete(
                    self.task_pos, col_ind[:self.m], axis=0)     # 更新task
                self.n = self.n - self.m
                uavs_pos_record.append(self.agent_pos[:])

                # 更新代价矩阵
                max_mn = max(self.m, self.n)
                cost = np.zeros([max_mn, max_mn])
                for i in range(self.m):
                    for j in range(self.n):
                        cost[i][j] = np.linalg.norm(self.agent_pos[i]-self.task_pos[j])

            # 添加虚拟task，补齐cost矩阵
            for i in range(self.m):
                for j in range(self.n, self.m):
                    cost[i][j] = 0x3f3f3f3f

            row_ind, col_ind = linear_sum_assignment(cost)
            tmp = np.zeros(self.agent_pos.shape)
            for i in range(self.m):
                if col_ind[i] < self.n:
                    tmp[i] = self.task_pos[col_ind[i]]   # 更新agent位置
                else:
                    tmp[i] = self.agent_pos[i]
            # self.agent_pos = self.task_pos[col_ind[:self.m]]
            uavs_pos_record.append(tmp)
        
        else:
            k = self.m // self.n
            tmp = np.zeros(self.agent_pos.shape)
            for t in range(k):
                row_ind, col_ind = linear_sum_assignment(cost[t*self.n:(t+1)*self.n,:self.n])
                tmp[t*self.n:(t+1)*self.n] = self.task_pos[col_ind[:]]

            if self.m%self.n != 0:
                cost_res = np.zeros([self.n, self.n])
                cost_res[:self.m%self.n] = cost[k*self.n:, :self.n]
                cost_res[self.m%self.n:] = 0x3f3f3f3f * np.ones([self.n-self.m%self.n, self.n])
                row_ind, col_ind = linear_sum_assignment(cost_res)
                tmp[k*self.n:] = self.task_pos[col_ind[:self.m%self.n]]

            self.agent_pos = tmp
            uavs_pos_record.append(tmp)

        return uavs_pos_record


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    task_pos = np.array([(-19, -4, 0), (-1, -1, -1), (-3, 13, 0), (14, -15, 0), (16, 0, 0)])
    agent_pos = np.array([(-13, 8, 0), (-12, 20, 0), (4, 4, 0), (18, -12, 0), (4, -19, 0),
                          (-19, 11, 0), (19, -8, 0), (-1, 9, 0), (-9, -8, 0), (11, -6, 0),
                          (-18, -17, 0), (-7, -16, 0), (12, 4, 0), (7, -1, 0)])
    m, n = len(agent_pos), len(task_pos)

    r = RHA2(agent_pos, task_pos)
    uavs_pos_record = r.deal()
    print(uavs_pos_record)
    path_len = len(uavs_pos_record)
    path_x = np.empty([m, path_len])
    path_y = np.empty([m, path_len])
    for i in range(path_len):
        for j in range(m):
            path_x[j][i] = uavs_pos_record[i][j][0]
            path_y[j][i] = uavs_pos_record[i][j][1]
    for i in range(m):
        plt.plot(path_x[i], path_y[i])
    for i in range(m):
        plt.annotate('U',
                     xy=(uavs_pos_record[0][i][0],
                         uavs_pos_record[0][i][1]), xycoords='data',
                     xytext=(0, +5), textcoords='offset points', fontsize=10)
    plt.show()