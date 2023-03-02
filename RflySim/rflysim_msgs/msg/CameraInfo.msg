std_msgs/Header header
int32 seq_id
int32 data_width
int32 data_height
int32 FOV #单位为degree

#PTZ(云台当前位姿,相对于载体,需要根据载体坐标系赋予对应的值)
float64 x
float64 y
float64 z
float64 yaw
float64 pitch
float64 roll

#相机内参矩阵
float64[] D
float64[9] K
float64[9] R
float64[12] P
