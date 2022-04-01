#!/bin/bash

. ../devel/setup.bash

# 获取飞机ID，设置总飞机数
MAVID=`expr ${HOSTNAME:4:2} + 0`
MAVNUM=3

echo "this MAV id :${MAVID}"



# RflySim ROS接口
roslaunch rflysim_sensor_rospkg rgb_newprotocol_cpp.launch  & PID0=$!
sleep 3s

# mavros and localization
roslaunch bs_assis bs_mavros.launch  mav_id:=${MAVID} & PID1=$!
roslaunch localization bias.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} & PID2=$!
sleep 5s

# DDS
roslaunch bs_assis bs_dds.launch  mav_id:=${MAVID} mav_num:=${MAVNUM} & PID3=$!
sleep 5s


wait
kill -9 PID0 PID1 PID2 PID3
exit