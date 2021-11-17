#!/usr/bin/env python
#coding=utf-8

import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, RCIn, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Point
from sensor_msgs.msg import Imu, NavSatFix, Temperature
from std_msgs.msg import Float32, Float64, String, Float32MultiArray, Int8
from nav_msgs.msg import Odometry
import time
import math
import numpy as np
import Tkinter
import threading
import inspect
import ctypes

class Px4Controller:
    def __init__(self, mode):
        self.mode = mode
        self.task_ready = False
        self.img_cnt = 0

        self.trigger_land = 0

        self.mavros_state = State()
        self.mav_pos = np.array([0., 0., 0.])
        self.mav_pos0 = np.array([0., 0., 0.])
        self.mav_vel = np.array([0., 0., 0.])
        self.mav_yaw = 0
        self.mav_yaw_0 = 0
        self.mav_R = np.identity(3)
        self.circle_xyr = [-1,-1,-1]
        self.circle_depth = 0
        self.kp_position = 1
        self.kd_position = 0
        self.k_yaw = 0.5

        self.arm_state = False
        self.offboard_state = False

        self.mavros_vel_ctl = True
        self.mavros_att_ctl = False

        self.command_vel =  construct_veltarget()
        Quaternion = [0,0,0,0]
        self.command_att = construct_AttYawrateThrottleTarget(Quaternion, 0, 0)
        Quaternion = [0,0,0,0]
        self.command_att = construct_AttYawrateTarget(Quaternion, 0)
        self.command_Vz = construct_VzTarget(0)
        self.command_vel =  construct_veltarget()

        self.ch5, self.ch6, self.ch7, self.ch8, self.ch9, self.ch11, self.ch14 = 0, 0, 0, 0, 0, 0, 0
        self.is_initialize_pos, self.is_initialize_vel, self.is_initialize_rc, self.is_initialize_img = False, False, False, False

        '''
        ros publishers
        '''
        # self.local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.local_vel_pub =  rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.att_pub =  rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.Vz_pub =  rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)

        self.trigger_pub = rospy.Publisher('fastplanner_trigger', Point, queue_size=10)
        # self.visual2fp_pub = rospy.Publisher('fastplanner_trigger', Int8, queue_size=1)
        self.log_pub = rospy.Publisher('python_log', Temperature, queue_size=10)
        self.land_trigger_mao=rospy.Publisher("/land_trigger_mao",Int8,queue_size=10)
        self.mavpos_cali_pub =rospy.Publisher("/log/mavpos_cali", PoseStamped, queue_size=10)
        self.statemachine_pub =rospy.Publisher("/log/statemachine", Int8, queue_size=1)
        self.mavpos_est_pub=rospy.Publisher("/log/mavpos_est", PoseStamped, queue_size=1)
        self.fastplanner_stop_pub = rospy.Publisher("/fastplanner_stop_trigger",Int8,queue_size=1)
        
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.local_vel_callback)
        self.pos_image_sub = rospy.Subscriber("tracker/pos_image", Float32MultiArray, self.pos_image_callback)
        self.semicircle_depth_sub = rospy.Subscriber("tracker/depth", Float32MultiArray, self.semicircle_depth_callback)
        # self.odometry_sub = rospy.Subscriber("camera/odom/sample", Odometry, self.odometry_callback)  # local_position易发散时使用
        self.mavros_sub = rospy.Subscriber("mavros/state", State, self.mavros_state_callback)
        self.rcin_sub = rospy.Subscriber("mavros/rc/in", RCIn, self.rcin_callback)
        self.fp2visual_sub  = rospy.Subscriber("/land_trigger", Int8, self.fp2visual_callback)
        
        # rospy.Subscriber("mp_state", Int8, self.mp_state_callback)
        

        if self.mode == "Simulation":
            self.inputThread = threading.Thread(target=self.read_kbd_input)
            self.inputThread.start()

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('/mavros/cmd/land')
        self.landService = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    
    def setToAttController(self):
        self.mavros_vel_ctl = False
        self.mavros_att_ctl = True
        
    def setToVelController(self):
        self.mavros_vel_ctl = True
        self.mavros_att_ctl = False  
                
    def mavros_pub(self):
        pub_time = time.time()
        while (not rospy.is_shutdown()):
            if self.mavros_vel_ctl and (time.time() - pub_time) > 1/50.0:
                self.local_vel_pub.publish(self.command_vel)
                pub_time = time.time()
            if self.mavros_att_ctl and (time.time() - pub_time) > 1/50.0:
                self.att_pub.publish(self.command_att)
                # self.Vz_pub.publish(self.command_Vz)
                pub_time = time.time()
            time.sleep(0.01)
                
        self.stop_thread(self.thread_mavros_pub)
        self.stop_thread(self.inputThread)


    def start_pub(self):
        self.thread_mavros_pub = threading.Thread(target=self.mavros_pub, args=())
        self.thread_mavros_pub.start()
        time.sleep(1)
        print("start pub vel")

    def stop_thread(self, thread_id):
        _async_raise(thread_id, SystemExit)
        time.sleep(1)
        print("a thread  is dead")
        
    
    def takeoff(self, vz=0.8,h=1.8):
        self.setToVelController()
        '''
            直接给速度，不会清零
        '''
        self.mav_yaw_0 = self.mav_yaw
        if not self.offboard_state:
            print("not in offboard, switch in offboard mannualy")
            return

        if self.arm_state:
            print("mav is in sky")
            return
        self.arm()



        self.command_vel = construct_veltarget(U=vz)
        takeoff_done = False
        while not takeoff_done:
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = self.mav_pos[2] > h
        self.command_vel = construct_veltarget()

    def takeoff_pos(self,h=1.8):
        self.setToVelController()
        '''
            直接给速度，不会清零
        '''
        self.mav_yaw_0 = self.mav_yaw
        if not self.offboard_state:
            print("not in offboard, switch in offboard mannualy")
            return

        if self.arm_state:
            print("mav is in sky")
            return
        self.arm()

        # self.command_vel = construct_veltarget(U=vz)
        self.moveByPosENU(U=h)
        takeoff_done = False
        while not takeoff_done:
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = (abs(self.mav_pos[2]- h) < 0.2)
        self.command_vel = construct_veltarget()
    
    def takeoff_fast(self,h=1.8):
        self.setToVelController()
        '''
            直接给速度，不会清零
        '''
        self.mav_yaw_0 = self.mav_yaw
        if not self.offboard_state:
            print("not in offboard, switch in offboard mannualy")
            return

        if self.arm_state:
            print("mav is in sky")
            return
        self.arm()

        # self.command_vel = construct_veltarget(U=vz)
        self.moveByPosENU(U=h)
        takeoff_done = False
        while not takeoff_done:
            # print("takeoff height:",{self.mav_pos[2]})
            takeoff_done = (self.mav_pos[2] > h*0.85)
        self.command_vel = construct_veltarget()
    
    def land(self):
        try:
            #http://wiki.ros.org/mavros/CustomModes for custom modes
            isLanding = self.landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except:
            print("land error!!!")
        # if self.flightModeService(custom_mode='AUTO.LAND'):
        #     return True
        # else:
        #     print("Vechile AUTO.LAND failed")
        #     return False

    def local_pose_callback(self, msg):
        if not self.is_initialize_pos:
            self.mav_yaw_0 = self.mav_yaw
        self.is_initialize_pos = True
        self.mav_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if self.task_ready:
            self.mav_pos -= self.mav_pos0
        q0, q1, q2, q3 = msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z
        self.Quaternion = np.array([q0, q1, q2, q3])
        self.mav_yaw = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3))
        self.mav_R = np.array([
            [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)],
            [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
            [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
        ])

        msg_cali = PoseStamped()
        msg_cali.header = msg.header
        msg_cali.pose.position.x, msg_cali.pose.position.y, msg_cali.pose.position.z = self.mav_pos
        msg_cali.pose.orientation = msg.pose.orientation
        self.mavpos_cali_pub.publish(msg_cali)
        # print("mav_pos: {}, mav_vel: {}, mav_yaw: {}, mav_R: {}".format(self.mav_pos, self.mav_vel, self.mav_yaw, self.mav_R))


    def local_vel_callback(self, msg):
        self.is_initialize_vel = True
        self.mav_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    # Calculates Rotation Matrix given euler angles.
    def eulerAnglesToRotationMatrix(self, theta) :
        R_x = np.array([[1,                  0,                   0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0,  math.sin(theta[0]), math.cos(theta[0]) ]])
                            
        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0,                  1,                  0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]])
                    
        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]),  0],
                        [0,                  0,                   1]])
                        
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def odometry_callback(self, msg):
        self.is_initialize_pos = True
        self.is_initialize_vel = True
        self.mav_pos_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.mav_vel_odom = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        q0, q1, q2, q3 = msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        self.mav_yaw_odom = math.atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3)) + self.mav_yaw_0
        self.mav_roll_odom = math.atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2))
        self.mav_pitch_odom = math.asin(2*(q0*q2 - q3*q1)) - math.pi/4
        self.mav_R_odom = self.eulerAnglesToRotationMatrix([self.mav_roll_odom, self.mav_pitch_odom, self.mav_yaw_odom])
        print("mav_pos_odom: {}, mav_vel_odom: {}, mav_yaw_odom: {}, mav_R_odom: {}".format(self.mav_pos_odom, self.mav_vel_odom, self.mav_yaw_odom, self.mav_R_odom))

    def pos_image_callback(self, msg):
        self.is_initialize_img = True
        if msg.data[2] > 0:
            self.img_cnt += 1
            if self.img_cnt >= 2:
                self.circle_xyr = msg.data
            else:
                self.circle_xyr = [0., 0., 0.]
        else:
            self.img_cnt = 0
            self.circle_xyr = msg.data
        # self.circle_xyr = msg.data

    def semicircle_depth_callback(self, msg):
        self.circle_depth=msg.data[0]

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        self.arm_state = msg.armed
        self.mode_str = msg.mode
        self.offboard_state = True if msg.mode == "OFFBOARD" else False

    def rcin_callback(self, msg):
        self.is_initialize_rc = True
        last_ch5, last_ch6, last_ch7, last_ch8, last_ch9, last_ch11, last_ch14 = self.ch5, self.ch6, self.ch7, self.ch8, self.ch9, self.ch11, self.ch14
        chs = msg.channels
        # 0-1300->0/1301-1700->1/1701-2000->2
        self.ch5 = 0 if chs[4] < 1300 else 1 if chs[4] < 1700 else 2
        self.ch6 = 0 if chs[5] < 1300 else 1 if chs[5] < 1700 else 2
        self.ch7 = 0 if chs[6] < 1300 else 1 if chs[6] < 1700 else 2
        self.ch8 = 0 if chs[7] < 1300 else 1 if chs[7] < 1700 else 2
        self.ch9 = 0 if chs[8] < 1300 else 1 if chs[8] < 1700 else 2
        self.ch11 = 0 if chs[10] < 1500 else 1
        self.ch14 = 0 if chs[10] < 1500 else 1
        if self.ch5!=last_ch5 or self.ch6!=last_ch6 or self.ch7!=last_ch7 or self.ch8!=last_ch8 or self.ch9!=last_ch9 or self.ch11!=last_ch11 or self.ch14!=last_ch14:
            print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}, ch11: {}, ch14: {}".format(self.ch5, self.ch6, self.ch7, self.ch8, self.ch9, self.ch11, self.ch14))

    # 
    def fp2visual_callback(self, msg):
        self.trigger_land = msg.data
    
    # def land_trigger_pub(self):
    #     for i in range(1,10):
    #         self.land_trigger_mao.publish(Int8(1))

    def call(self, event):
        k = event.keysym
        if k == "m":
            self.ch6 = 0
        elif k == "o":
            self.ch8 = 1
        elif k == "p":
            self.ch8 = 0
        elif k == "c":
            self.ch9 = (self.ch9 + 1) % 2
        elif k == "a":
            self.ch7 = 2
        elif k == "b":
            self.ch7 = 0
        print("ch5: {}, ch6: {}, ch7: {}, ch8: {}, ch9: {}, ch11: {}, ch14: {}".format(self.ch5, self.ch6, self.ch7, self.ch8, self.ch9, self.ch11, self.ch14))
        time.sleep(0.02)

    def read_kbd_input(self):
        self.is_initialize_rc = True
        win = Tkinter.Tk()
        frame = Tkinter.Frame(win,width=100,height=60)
        frame.bind("<Key>", self.call)
        frame.focus_set()
        frame.pack()
        win.mainloop()

    def FLU2ENU(self, x, y, z):
        ENU_x = x * math.cos(self.mav_yaw) - y * math.sin(self.mav_yaw)
        ENU_y = x * math.sin(self.mav_yaw) + y * math.cos(self.mav_yaw)
        ENU_z = z
        return ENU_x, ENU_y, ENU_z

    def moveByVelocityYawrateENU(self, E=0, N=0, U=0, yaw_rate=0):
        self.setToVelController()
        self.command_vel = construct_veltarget_ENU(E, N, U,yaw_rate)

    def moveByRollPitchYawrateThrottle(self, Quaternion, yaw_rate_cmd, throttle):
        self.setToAttController()
        self.command_att = construct_AttYawrateThrottleTarget(Quaternion, yaw_rate_cmd, throttle)

    def moveByRollPitchYawrateVz(self, Quaternion, yaw_rate_cmd, Vz):
        self.setToAttController()
        # self.command_att = construct_AttYawrateTarget(Quaternion, yaw_rate_cmd)
        self.command_Vz = construct_VzTarget(Vz)
        
    def moveByAccYawrate(self, acc, yaw_rate_cmd):
        self.setToVelController()
        # self.command_att = construct_AttYawrateTarget(Quaternion, yaw_rate_cmd)
        self.command_vel = construct_AccTarget(acc, yaw_rate_cmd)
        
    def moveByVelocityYawrateBodyFrame(self, vx, vy, vz, yaw_rate=0):
        self.setToVelController()
        """
        - function: 线速度加偏航角速度接口
        - params:
            - vx (float): desired velocity in the X axis of the vehicle's local FLU frame.
            - vy (float): desired velocity in the Y axis of the vehicle's local FLU frame.
            - vz (float): desired velocity in the Z axis of the vehicle's local FLU frame.
            - wyaw (float): desired yaw rate in rad
        """
        self.command_vel = construct_veltarget(vx, vy, vz, yaw_rate)
    
    def moveByPosENU(self, E=None, N=None, U=None, mav_yaw=None):
        self.setToVelController()
        mav_yaw = mav_yaw if mav_yaw is not None else self.mav_yaw
        E = E if E is not None else self.mav_pos[0]
        N = N if N is not None else self.mav_pos[1]
        U = U if U is not None else self.mav_pos[2]
        self.command_vel = construct_postarget_ENU(E, N, U, mav_yaw)
    
    # ENU下的角yaw_rad
    def moveByYaw(self, yaw_rad):
        self.setToVelController()
        E,N,U = self.mav_pos
        self.command_vel = construct_postarget_ENU(E,N,U,yaw_rad)

    def moveToPositionOnceAsync(self, x, y, z, yaw_d, velocity=1):
        """
        - function: 向world ENU坐标系下期望位置和偏航角飞行
        - params:
            - x (float): desired position in world (ENU) X axis
            - y (float): desired position in world (ENU) Y axis
            - z (float): desired position in world (ENU) Z axis
            - yaw_d (float): desired yaw
            - velocity (float): velocity saturation
        - return:
            - 异步执行
        - usage:
            while d < th: update p_d, p  moveToPositionOnceAsync(x, y, z, yaw, v)  time.sleep(t)
        """
        p_d = np.array([x, y, z])
        cmd_v = self.saturation(self.kp_position * (p_d - self.mav_pos) + self.kd_position*(-self.mav_vel), velocity)
        cmd_yaw = self.saturation(self.k_yaw * self.minAngleDiff(yaw_d, self.mav_yaw), 0.4)
        self.moveByVelocityYawrateENU(cmd_v[0], cmd_v[1], cmd_v[2], cmd_yaw)

    def moveToPosition(self, x, y, z, yaw_d=None, velocity=1):
        p_d = np.array([x, y, z])
        if yaw_d is None:
            yaw_d = self.mav_yaw
        while ( np.linalg.norm(p_d - self.mav_pos) > 0.3 or 
                np.linalg.norm(self.mav_vel) > 0.2 or 
                abs(self.minAngleDiff(yaw_d, self.mav_yaw)) > 0.1
            ):
            # print(p_d - self.mav_pos)
            self.moveToPositionOnceAsync(p_d[0], p_d[1], p_d[2], yaw_d, velocity)
            
            time.sleep(0.02)
        self.moveByVelocityYawrateENU()   
            

    def saturation(self, a, maxv):
        n = np.linalg.norm(a)
        if n > maxv:
            return a / n * maxv
        else:
            return a

    def minAngleDiff(self, a, b):
        diff = a - b
        if diff < 0:
            diff += 2*np.pi
        if diff < np.pi:
            return diff
        else:
            return diff - 2*np.pi


    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        offb_set_mode = SetMode()
        offb_set_mode.custom_mode = "OFFBOARD"
        resp1 = self.flightModeService(0, offb_set_mode.custom_mode)
        if resp1.mode_sent:
            print("Offboard enabled")
            return True
        else:
            print("Vechile Offboard failed")
            return False

    def logging(self, val1, val2=0):
        logging_data = Temperature()
        logging_data.temperature = val1
        logging_data.variance = val2
        self.log_pub.publish(logging_data)

    def planning(self, tx, ty, tz):
        for i in range(10):
            self.trigger_pub.publish(Point(tx, ty, tz))
            time.sleep(0.05)
    
    def planning_stop(self):
        for i in range(5):
            self.fastplanner_stop_pub.publish(1)
            time.sleep(0.01)

    def idle(self):
        self.mavros_vel_ctl = False
        self.mavros_att_ctl = False
        print("idle")

    def enable(self):
        self.mavros_vel_ctl = True
        self.mavros_att_ctl = False
        print("enable")

# 构造FLU的速度目标
def construct_veltarget(F=0, L=0, U=0, yaw_rate=0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()
    
    '''
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9
    '''
    target_raw_pose.coordinate_frame = 8

    target_raw_pose.velocity.x = F
    target_raw_pose.velocity.y = L
    target_raw_pose.velocity.z = U

    target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE + PositionTarget.IGNORE_YAW 

    target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose


def construct_veltarget_ENU(E=0, N=0, U=0, yaw_rate=0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()
    
    '''
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9
    '''
    # 直接就ENU了
    target_raw_pose.coordinate_frame = 1

    target_raw_pose.velocity.x = E
    target_raw_pose.velocity.y = N
    target_raw_pose.velocity.z = U

    target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE + PositionTarget.IGNORE_YAW 

    target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose


def construct_postarget_ENU(E=0, N=0, U=0, yaw=0, yaw_rate = 0):
    target_raw_pose = PositionTarget()
    target_raw_pose.header.stamp = rospy.Time.now()
    
    '''
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9

    MAV_FRAME_BODY_FRD
    '''

    target_raw_pose.coordinate_frame = 1

    target_raw_pose.position.x = E
    target_raw_pose.position.y = N
    target_raw_pose.position.z = U

    target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                + PositionTarget.FORCE

    target_raw_pose.yaw = yaw
    target_raw_pose.yaw_rate = yaw_rate
    return target_raw_pose

def construct_AttYawrateThrottleTarget(Quaternion, yaw_rate, thrust):
    target_att = AttitudeTarget()
    
    target_att.type_mask = 3
    
    target_att.orientation.x = Quaternion[0]
    target_att.orientation.y = Quaternion[1]
    target_att.orientation.z = Quaternion[2]
    target_att.orientation.w = Quaternion[3]
    
    target_att.body_rate.z = yaw_rate
    
    target_att.thrust = thrust
    return target_att

    
def construct_AttYawrateTarget(Quaternion, yaw_rate):
    target_att = AttitudeTarget()
    
    target_att.type_mask = 67
    
    target_att.orientation.x = Quaternion[0]
    target_att.orientation.y = Quaternion[1]
    target_att.orientation.z = Quaternion[2]
    target_att.orientation.w = Quaternion[3]
    
    target_att.body_rate.z = yaw_rate
    return target_att

def construct_VzTarget(Vz):
    target_Vz = GlobalPositionTarget()
    
    target_Vz.coordinate_frame = 6 
    target_Vz.type_mask = 1+2+4+8+16+64+128+256+512+1024+2048
    
    target_Vz.velocity.z = Vz
    
    return target_Vz

def construct_AccTarget(acc=[0,0,0], yaw_rate_cmd=0):
    target_acc = PositionTarget()
    
    target_acc.coordinate_frame = 1 
    target_acc.type_mask = 1+2+4+8+16+32+1024
    
    target_acc.acceleration_or_force.x = acc[0]
    target_acc.acceleration_or_force.y = acc[1]
    target_acc.acceleration_or_force.z = acc[2]

    target_acc.yaw_rate = yaw_rate_cmd

    
    return target_acc
    
def _async_raise(tid, exctype):
    
    if not inspect.isclass(exctype):
        raise TypeError("Only types can be raised (not instances)")
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

if __name__ == '__main__':
    con = Px4Controller("Simulation")
    con.start()