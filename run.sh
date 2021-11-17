#!/bin/bash

. ../devel/setup.bash

# 获取飞机ID，设置总飞机数
MAVID=`expr ${HOSTNAME:4:2} + 0`
MAVNUM=3

echo "this MAV id :${MAVID}"


# RflySim ROS接口
roslaunch rflysim_sensor_rospkg rgb_newprotocol_cpp.launch  & PID0=$!
sleep 5s

# 目标检测和椭圆检测
rosrun detection img_pub.py  & PID1=$!
sleep 5s

# DDS
roslaunch bs_assis bs_dds.launch  mav_id:=${MAVID} mav_num:=${MAVNUM} & PID2=$!
roslaunch bs_assis bs_mavros.launch  mav_id:=${MAVID} & PID6=$!
sleep 5s

# 控制汇总
roslaunch offboard_pkg assemble.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID3=$!
sleep 5s

# 决策相关
roslaunch decision multi_drone_bs.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} & PID4=$!
sleep 10s

# 打击
roslaunch attack attack.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID5=$!
sleep 5s


wait
kill -9 PID0 PID1 PID2 PID3 PID4 PID5 PID6
exit