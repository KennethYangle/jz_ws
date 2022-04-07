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

# 可视化
roslaunch visiualization visual.launch  & PID4=$!
sleep 3s

# 目标检测
rosrun detection img_pub.py  & PID5=$!
sleep 3s

# 控制汇总
roslaunch offboard_pkg assemble.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID6=$!
sleep 5s

# 决策相关
roslaunch decision drone.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} & PID7=$!
sleep 1s
roslaunch decision_maker simu_h.launch  & PID8=$!
sleep 1s
roslaunch decision_maker one_uav_h.launch  & PID9=$!
sleep 5s

# 打击
roslaunch attack attack.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID10=$!
sleep 5s

# 管道生成
roslaunch tube_planning tube_plan.launch & PID11=$!
sleep 5s

# 管道飞行
roslaunch tube_control tube.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID12=$!
sleep 5s

wait
kill -9 PID0 PID1 PID2 PID3 PID4 PID5 PID6 PID7 PID8 PID9 PID10 PID11 PID12
exit