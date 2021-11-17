#!/usr/bin/env python
# coding=utf-8

# 本程序为决策模块主要实现。负责调度任务全流程，从剧本中拿到管道并与集群管道控制器交互

import rospy
import os, sys
import json
import threading
import numpy as np

from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped, Point32
from std_msgs.msg import UInt64



if __name__ == '__main__':
    # 载入剧本文件
    # file_path = os.path.join(os.path.expanduser('~'),"Swarm_ws/src/decision/scenarios","play_2directions_12drones.json")
    # play_file = open(file_path)
    # play = json.load(play_file)
    # # print(json.dumps(play))

    # ROS初始化，从launch文件获取参数
    rospy.init_node('decision_node', anonymous=True)
    drone_id = int(rospy.get_param("~drone_id"))

    # # 判断是否有对应的剧本
    # if drone_name not in play.keys():
    #     print("Name Error! {} not in {}".format(drone_name, file_path))
    #     sys.exit(1)
    



    # 待添加 -----------------------------------------
    # 绕飞、集群飞行的路径点发布

