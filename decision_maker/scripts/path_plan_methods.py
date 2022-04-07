#! /usr/bin/env python3
# coding=utf-8


from math import sqrt
from bresenham import bresenham_line
# import numpy as np


def standardize_input(func):
    def wrapper(*args):
        args = [args[0]] + list(map(lambda x: tuple(x), args[1:]))
        return func(*args)
    return wrapper


class LazyThetaStar:
    def __init__(self, grid_map, puffy_size=1, obs_threshold=0.5):
        self.grid_map = grid_map
        self.puffy_size = puffy_size
        self.obs_threshold = obs_threshold

        self.xr = range(- self.puffy_size, self.puffy_size + 1)
        self.yr = range(- self.puffy_size, self.puffy_size + 1)

        self.open_set = set()  # set(tuple(pos))
        self.close_set = set()  # set(tuple(pos))
        self.pos_gh_dict = dict()  # tuple(pos) -> [g(pos), h(pos)]
        self.pos_parent_dict = dict()  # tuple(pos) -> parent(pos)

        self.neighbor_list = []  # [tuple(pos)]

    @staticmethod
    def distance_p(pos1, pos2):
        return sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    @standardize_input
    def path_plan(self, start_pos, goal_pos):
        # self.binary_map = np.where(self.grid_map > self.obs_threshold, 1, 0)
        self.open_set.clear()
        self.close_set.clear()
        self.pos_gh_dict.clear()
        self.pos_parent_dict.clear()

        self.open_set.add(start_pos)
        self.pos_gh_dict[start_pos] = [0, self.distance_p(start_pos, goal_pos)]  # [g(pos), h(pos)]
        self.pos_parent_dict[start_pos] = start_pos

        while self.open_set:
            current_pos = min(self.open_set, key=lambda x: sum(self.pos_gh_dict[x]))
            self.open_set.remove(current_pos)
            self.set_neighbors(current_pos)
            self.set_vertex(current_pos)
            if current_pos == goal_pos:
                result_list = []
                while current_pos != self.pos_parent_dict[current_pos]:
                    result_list.append(current_pos)
                    current_pos = self.pos_parent_dict[current_pos]
                if not result_list:  # 有路径且起始点等于目标点，放入目标点 
                    result_list.append(goal_pos)
                return result_list
            self.close_set.add(current_pos)
            for n_point in self.neighbor_list:
                if n_point not in self.close_set:
                    if n_point not in self.open_set:
                        self.pos_gh_dict[n_point] = [float('inf'), 0]
                        self.pos_parent_dict[n_point] = None
                    self.update_vertex(current_pos, n_point, goal_pos)
        # print('NO PATH!!!')
        return [start_pos]

    def set_neighbors(self, current_pos):
        self.neighbor_list.clear()
        for delta_x in [-1, 0, 1]:
            for delta_y in [-1, 0, 1]:
                if not delta_x and not delta_y:
                    continue
                neighbor = (current_pos[0] + delta_x, current_pos[1] + delta_y)
                if neighbor[0] < 0 or neighbor[1] < 0:
                    continue
                try:
                    grid_state = self.grid_map[neighbor[0]][neighbor[1]]
                except IndexError:
                    continue
                else:
                    if grid_state > self.obs_threshold:
                        continue
                    else:
                        self.neighbor_list.append(neighbor)

    def set_vertex(self, current_pos):
        if self.visibility_judge(current_pos, self.pos_parent_dict[current_pos]):
            min_g = float('inf')
            argmin_g = self.pos_parent_dict[current_pos]
            for point in self.neighbor_list:
                if point in self.close_set:
                    _g = self.pos_gh_dict[point][0] + self.distance_p(point, current_pos)
                    if _g < min_g:
                        min_g = _g
                        argmin_g = point
            self.pos_gh_dict[current_pos][0] = min_g
            self.pos_parent_dict[current_pos] = argmin_g

    def visibility_judge(self, pos1, pos2):
        # return np.sum(self.binary_map[min(pos1[0], pos2[0]): max(pos1[0], pos2[0]), min(pos1[1], pos2[1]): max(pos1[1], pos2[1])])
        line_grid = bresenham_line(pos1, pos2)
        for pos in line_grid:
            if self.grid_map[pos[0], pos[1]] > self.obs_threshold:
                return True
        return False

    def puffy_judge(self, pos):
        for dx in self.xr:
            for dy in self.yr:
                try:
                    if self.grid_map[pos[0]+dx][pos[1]+dy] > self.obs_threshold:
                        return False
                except IndexError:
                    return False
        return True

    def update_vertex(self, current, neighbor, goal):
        g_old = self.pos_gh_dict[neighbor][0]
        self.compute_cost(current, neighbor)
        if self.pos_gh_dict[neighbor][0] < g_old:# and self.puffy_judge(neighbor):
            self.open_set.add(neighbor)
            self.pos_gh_dict[neighbor][1] = self.distance_p(neighbor, goal)

    def compute_cost(self, current, neighbor):
        parent_c = self.pos_parent_dict[current]
        g_new = self.pos_gh_dict[parent_c][0] + self.distance_p(parent_c, neighbor) 
        if g_new < self.pos_gh_dict[neighbor][0]:
            self.pos_parent_dict[neighbor] = parent_c
            self.pos_gh_dict[neighbor][0] = g_new
 

if __name__ == '__main__':
    import time
    import random
    import matplotlib.pyplot as plt
    import numpy as np
    # from simulation_environment import generate_gridmap
    from simu_data_h import generate_map

    grid_map0 = generate_map(1)
    obs_threshold0 = 70
    # binary_map0 = np.where(grid_map0 > obs_threshold0, 1, 0)
    map_shape = grid_map0.shape
    while True:
        start_p = (random.choice(range(map_shape[0])), random.choice(range(map_shape[1])))
        if grid_map0[start_p[0]][start_p[1]] < obs_threshold0:
            while True:
                goal_p = (random.choice(range(map_shape[0])), random.choice(range(map_shape[1])))
                if grid_map0[goal_p[0]][goal_p[1]] < obs_threshold0:
                    break
            break
    # start_p = (16, 358)
    # goal_p = (135, 7)
    print(start_p, goal_p)
    solver = LazyThetaStar(grid_map0, obs_threshold=obs_threshold0)
    start = time.time()
    path_list = solver.path_plan(start_p, goal_p)  # 按从目标点到起始点的顺序
    end = time.time()
    path_list.append(start_p)
    path_pos_l = [goal_p]
    for i in range(len(path_list) - 1):
        p1 = path_list[i]
        p2 = path_list[i + 1]
        path_pos_l += bresenham_line(p1, p2)
    for p in path_pos_l:
        grid_map0[p[0], p[1]] = 200
    for p in path_list:
        grid_map0[p[0], p[1]] = 300
    plt.matshow(grid_map0, cmap=plt.cm.gray_r)
    plt.show()
    print(path_list)
    print(path_pos_l)
    print(end-start)

