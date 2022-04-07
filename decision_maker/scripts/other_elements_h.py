#! /usr/bin/env python3
# coding=utf-8


class Target:
    def __init__(self, tar_id, position, area, moving=False):
        self.tar_id = tar_id
        self.position = position
        self.moving = moving
        self.area = area

    def __eq__(self, other):
        return self.tar_id == other.tar_id


class Environment:
    def __init__(self, bound_x=0, bound_y=0):
        self.grid_map = []
        self.grid_resolution = 0
        self.org_in_global = ()
        self.height = 1.5

    # def real2grid(self, point):
    #     return (int((point[0] - self.org_in_global[0]) / self.grid_resolution), int((point[1] - self.org_in_global[1]) / self.grid_resolution))

    # def grid2real(self, point):
    #     return ((point[0] + 0.5) * self.grid_resolution + self.org_in_global[0], (point[1] + 0.5) * self.grid_resolution + self.org_in_global[1])

    def real2grid(self, point):
        return [int((point[1] - self.org_in_global[1]) / self.grid_resolution), int((self.org_in_global[0] - point[0]) / self.grid_resolution)]

    def grid2real(self, point):
        return [self.org_in_global[0] - (point[1] + 0.5) * self.grid_resolution, (point[0] + 0.5) * self.grid_resolution + self.org_in_global[1]]
