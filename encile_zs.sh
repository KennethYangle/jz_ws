#!/bin/bash

. ../devel/setup.bash

# 获取飞机ID，设置总飞机数
MAVID=`expr ${HOSTNAME:4:2} + 0`
#MAVID=1
MAVNUM=3
# RflySim仿真参数
UE4IP="192.168.3.1"
MAVX="0"
MAVY="0"
MAVZ="0"

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

# RflySim显示管道和创建物体接口
roslaunch visiualization single_rfly_obj_adder.launch mav_id:=${MAVID} ue4_ip:=${UE4IP} & PID4=$!
# roslaunch attack target_uav.launch mav_id:=${MAVID} mav_x:=${MAVX} mav_y:=${MAVY} mav_z:=${MAVZ} & PID5=$!
# sleep 2s

# # 可视化
# roslaunch visiualization visual.launch  & PID4=$!
# sleep 3s

# 目标检测
# rosrun detection img_pub.py  & PID5=$!
sleep 3s

# 控制汇总
roslaunch optical_flow zs.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID6=$!
# sleep 2s

# 决策相关
# roslaunch decision drone.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} & PID7=$!
# sleep 1s

# 打击
roslaunch optical_flow of_encile.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} & PID11=$!
sleep 2s
# roslaunch attack attack.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID10=$!
# sleep 3s
# 管道飞行

wait
kill -9 PID0 PID1 PID2 PID3 PID4 PID5 PID6 PID7 PID8 PID9 PID10 PID11 PID12
echo "kill done"
exit