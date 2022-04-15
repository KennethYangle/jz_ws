#!/bin/bash

. ../devel/setup.bash

# 获取飞机ID，设置总飞机数
MAVID=`expr ${HOSTNAME:4:2} + 0`
MAVNUM=3
# RflySim仿真参数
UE4IP="192.168.3.1"
MAVX="-144"
MAVY="43"
MAVZ="0"
VEHICLE_INTERVAL=1

echo "this MAV id :${MAVID}"


# RflySim ROS接口
roslaunch rflysim_sensor_rospkg rgb_newprotocol_cpp.launch  & PID0=$!
sleep 3s

# mavros and localization
roslaunch bs_assis bs_mavros.launch  mav_id:=${MAVID} & PID1=$!
roslaunch localization bias.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} vehicle_interval:=${VEHICLE_INTERVAL} & PID2=$!
sleep 5s

# DDS
roslaunch bs_assis bs_dds.launch  mav_id:=${MAVID} mav_num:=${MAVNUM} & PID3=$!
sleep 5s

# RflySim显示管道和创建物体接口
roslaunch visiualization single_rfly_obj_adder.launch mav_id:=${MAVID} ue4_ip:=${UE4IP} & PID4=$!
roslaunch visiualization show_pipline.launch mav_id:=${MAVID} mav_x:=${MAVX} mav_y:=${MAVY} mav_z:=${MAVZ} & PID5=$!
sleep 2s

# # 可视化
# roslaunch visiualization visual.launch  & PID6=$!
# sleep 3s

# 目标检测
rosrun detection img_pub.py  & PID7=$!
sleep 3s

# 控制汇总
roslaunch offboard_pkg assemble.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID8=$!
sleep 5s

# 决策相关
roslaunch decision drone.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM} & PID9=$!
sleep 1s
roslaunch decision_maker visual.launch  & PID10=$!
sleep 1s
roslaunch decision_maker simu_h.launch  uav_num:=${MAVNUM} & PID11=$!
sleep 1s
roslaunch decision_maker one_uav_h.launch  uav_id:=`expr ${MAVID} - 1` & PID12=$!
sleep 5s

# 打击
roslaunch attack attack.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID13=$!
sleep 5s

# 管道生成
roslaunch tube_planning tube_plan.launch  uav_id:=`expr ${MAVID} - 1` & PID14=$!
sleep 5s

# 管道飞行
roslaunch tube_control tube.launch  drone_id:=${MAVID}  drone_num:=${MAVNUM}  & PID15=$!
sleep 5s

wait
kill -9 PID0 PID1 PID2 PID3 PID4 PID5 PID6 PID7 PID8 PID9 PID10 PID11 PID12 PID13 PID14 PID15
exit