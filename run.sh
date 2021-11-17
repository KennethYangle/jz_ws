#!/bin/bash

. ../devel/setup.bash


MAVID=`expr ${HOSTNAME:4:2} + 0`
MAVNUM=3

echo "this MAV id :${MAVID}"

roslaunch rflysim_sensor_rospkg rgb_newprotocol_cpp.launch  & PID0=$!
sleep 5s

# 目标检测和椭圆检测,取消注释即可启用
# roslaunch shape_detection  obj_det.launch   & PID4=$!
# sleep 5s

# roslaunch shape_detection  ellipse_det.launch   & PID5=$!
# sleep 5s

roslaunch bs_assis bs_dds.launch  mav_id:=${MAVID} mav_num:=${MAVNUM} & PID1=$!
sleep 5s

roslaunch decision multi_drone_bs.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} & PID2=$!
sleep 5s

wait
kill -9 PID0 PID1 PID2
exit