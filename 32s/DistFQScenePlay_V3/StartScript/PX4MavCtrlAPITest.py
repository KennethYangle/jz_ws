# -*- encoding: utf-8 -*-
'''
@File    :   PX4MavCtrlAPITest.py
@Time    :   2021/11/23 11:08:26
@Author  :   YuanXiaoshuai 
@Version :   1.0
@Contact :   znzx_32@outlook.com
'''

# here put the import lib

import time
import math
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../CommonAPI'))
import PX4MavCtrlV4 as PX4MavCtrl

from threading import *
import socket
import struct

#
AttackRes=[0]

#接收打击结果
class SocketRevAttack:
    def __init__(self):
        port2 = 5000
        port3 = 5001
        self.Port = [port2, port3]
        self.server_address = []
        self.server_socket = []
        for i in range(2):
            self.server_socket.append(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            self.server_socket[i].setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.server_address.append(('0.0.0.0', self.Port[i]))
            self.server_socket[i].bind(self.server_address[i])
        self.datasize = struct.calcsize('1f')
    def Recv_Data1(self):
        recvData, client = self.server_socket[0].recvfrom(100)
        if (len(recvData) != self.datasize):
            return 0
        else:
            self.Content1 = struct.unpack('1f', recvData)
            return 1
    def Recv_Data2(self):
        recvData, client = self.server_socket[1].recvfrom(100)
        if (len(recvData) != self.datasize):
            return 0
        else:
            self.Content2 = struct.unpack('1f', recvData)
            return 1
def recive_attack_res():
    while 1:
        iret = socketRevAttack.Recv_Data1()
        if(iret==1):
            AttackRes[0]=socketRevAttack.Content1[0]
            print("attack_res2:",AttackRes[0])
#一楼人物1 走动的人 id=305 
def move_man2():
    step=-133.5
    print("attack_res0:",AttackRes[0])
    while 1:
        if(AttackRes[0]==1):
            print("attack_res3:",AttackRes[0])
            mav.sendUE4PosScale(306,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            mav.sendUE4PosScale(305,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            break
        else :
            print("attack_res4:",AttackRes[0])
            while (step<-133.5 and AttackRes[0]!=1):
                #0.008
                step=step+0.07
                mav.sendUE4PosScale(305,10030,0,[step,50,-0.35],[0,0,(0)*math.pi],[1.5,1.5,1.5])
                #二楼人物4头套 id=336
                mav.sendUE4PosScale(306,152,0,[step+0.05,50,-0.35-2.4],[0,0,math.pi],[0.11,0.11,0.11])
                time.sleep(0.05)
            while (step>-140.5 and AttackRes[0]!=1):
                step=step-0.07
                mav.sendUE4PosScale(305,10030,0,[step,50,-0.35],[0,0,(1)*math.pi],[1.5,1.5,1.5])
                #二楼人物4头套 id=336
                mav.sendUE4PosScale(306,152,0,[step-0.05,50,-0.35-2.4],[0,0,math.pi],[0.11,0.11,0.11])
                time.sleep(0.05)
    time.sleep(0.5)
#二楼人物4  走动的人 id=335
def move_man():
    step=57
    while 1:
        if(AttackRes[0]==5):
            print("attack_res3:",AttackRes[0])
            mav.sendUE4PosScale(335,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            mav.sendUE4PosScale(336,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            break
        else :
            print("attack_res4:",AttackRes[0])
            while (step<73 ):
            #0.008
              step=step+0.1
              mav.sendUE4PosScale(335,3030,0,[-130.5,step,-5],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])
            #二楼人物4头套 id=336
              mav.sendUE4PosScale(336,152,0,[-130.5,step+0.05,-5-2.6],[0,0,math.pi],[0.11,0.11,0.11])
              time.sleep(0.05)
        while (step>42):
            step=step-0.1
            mav.sendUE4PosScale(335,3030,0,[-130.5,step,-5],[0,0,(3/2)*math.pi],[1.5,1.5,1.5])
            #二楼人物4头套 id=336
            mav.sendUE4PosScale(336,152,0,[-130.5,step-0.05,-5-2.6],[0,0,math.pi],[0.11,0.11,0.11])
            time.sleep(0.05)
    time.sleep(0.5)
#二楼人物1  走动的人 id=329
def move_man3():
    step=39.5
    while 1:
        if(AttackRes[0]==4):
            mav.sendUE4PosScale(330,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            mav.sendUE4PosScale(329,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            break  
        else:
          while step<50.5:
            #0.008
            step=step+0.1
            mav.sendUE4PosScale(329,7030,0,[-143,step,-5],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])
            #二楼人物4头套 id=330
            mav.sendUE4PosScale(330,152,0,[-143,step+0.05,-5-2.4],[0,0,math.pi],[0.11,0.11,0.11])
            time.sleep(0.05)
          while step>39.5:
            step=step-0.1
            mav.sendUE4PosScale(329,7030,0,[-143,step,-5],[0,0,(3/2)*math.pi],[1.5,1.5,1.5])
            #二楼人物4头套 id=330
            mav.sendUE4PosScale(330,152,0,[-143,step-0.05,-5-2.4],[0,0,math.pi],[0.11,0.11,0.11])
            time.sleep(0.05)
    time.sleep(0.5)
def f2_man4():
    while(1):
       print("attack_res0:",AttackRes[0])
       if(AttackRes[0]==3):
            print("attack_res3:",AttackRes[0])
            mav.sendUE4PosScale(331,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            mav.sendUE4PosScale(332,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
#一楼门1 移动门2 id=314
def move_door1():
    step=60.25
    while 1:
        while step<=60.25:
            #0.008
            step=step+0.005
            mav.sendUE4PosScale(314,41,0,[-144.75,step,-0.9],[0,0,(0)*math.pi],[1,1.5,1.3])       
            time.sleep(0.05)
        while step>59.55:
            step=step-0.005
            mav.sendUE4PosScale(314,41,0,[-144.75,step,-0.9],[0,0,(0)*math.pi],[1,1.5,1.3])    
            time.sleep(0.05)
    time.sleep(0.5)
#一楼门1 移动门2 id=347
def move_door2():
    step=63.15
    while 1:
        while step>=63.15:
            step=step-0.005
            mav.sendUE4PosScale(347,41,0,[-144.7,step,-0.9],[0,0,(1)*math.pi],[1,1.5,1.3])    
            time.sleep(0.05)
        while step<63.85:
            #0.008
            step=step+0.005
            mav.sendUE4PosScale(347,41,0,[-144.7,step,-0.9],[0,0,(1)*math.pi],[1,1.5,1.3])       
            time.sleep(0.05)
    time.sleep(0.5)
#二楼门1 移动门1 id=348
def move_door3():
    step=64.5
    while 1:
        while step>=64.5:
            step=step-0.01
            mav.sendUE4PosScale(348,2041,0,[-132.7,step,-5.15],[0,0,(3/2)*math.pi],[1,1,1])    
            time.sleep(0.05)
        while step<65.3:
            #0.008
            step=step+0.01
            mav.sendUE4PosScale(348,2041,0,[-132.7,step,-5.15],[0,0,(3/2)*math.pi],[1,1,1])       
            time.sleep(0.05)
    time.sleep(0.5)
#二楼门1 移动门2 id=381
def move_door4():
    step=68
    while 1:
        while step<=68:
            #0.008
            step=step+0.01
            mav.sendUE4PosScale(381,2041,0,[-132.7,step,-5.15],[0,0,(1/2)*math.pi],[1,1,1])       
            time.sleep(0.05)
        while step>67.2:
            step=step-0.01
            mav.sendUE4PosScale(381,2041,0,[-132.7,step,-5.15],[0,0,(1/2)*math.pi],[1,1,1])    
            time.sleep(0.05)
    time.sleep(0.5)

def f1_man2():
    #一楼人物2 id=307
    while(1):
        if(AttackRes[0]!=2):
            mav.sendUE4PosScale(307,30,0,[-130.5,63,-0.2],[0,0,(1)*math.pi],[1.5,1.5,1.5])
            #一楼人物2头套 id=308
            mav.sendUE4PosScale(308,152,0,[-130.6,63,-2.8],[0,0,math.pi],[0.11,0.11,0.11])
        else:
            mav.sendUE4PosScale(307,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            mav.sendUE4PosScale(308,153,0,[-130.5,50,-0],[0,0,math.pi],[0.001,0.001,0.001])
            break

if __name__ == '__main__':

    #Create a new MAVLink communication instance, UDP sending port (CopterSim’s receving port) is 20100
    mav = PX4MavCtrl.PX4MavCtrler(20100)

    # sendUE4Cmd: RflySim3D API to modify scene display style
    # Format: mav.sendUE4Cmd(cmd,windowID=-1), where cmd is a command string, windowID is the received window number (assuming multiple RflySim3D windows are opened at the same time), windowID =-1 means sent to all windows
    # Augument: RflyChangeMapbyName command means to switch the map (scene), the following string is the map name, here will switch all open windows to the grass map
    mav.sendUE4Cmd(b'RflyChangeMapbyName OldFactory1')    


    #方便切换视角的模型 二维码
    #一楼视角1
    mav.sendUE4PosScale(21,40,0,[-130,40,-4.6],[0,0,math.pi],[0.5,0.5,0.5])#视角
    #一楼视角2
    mav.sendUE4PosScale(22,40,0,[-130,60,-4.6],[0,0,math.pi],[0.5,0.5,0.5])#视角
    #二楼视角1
    mav.sendUE4PosScale(23,40,0,[-134,39,-10],[0,0,math.pi],[0.5,0.5,0.5])#视角
    #二楼视角2
    mav.sendUE4PosScale(24,40,0,[-134,70,-10],[0,0,math.pi],[0.5,0.5,0.5])#视角
    #二楼视角3
    mav.sendUE4PosScale(25,40,0,[-128.7,39,-10],[0,0,math.pi],[0.5,0.5,0.5])#视角
    #二楼视角4
    mav.sendUE4PosScale(26,40,0,[-150.7,39,-10],[0,0,math.pi],[0.5,0.5,0.5])#视角

    #新场景里面模块id从301开始
    #一楼场景
    #一楼桌子
    #一楼桌子1 id=301
    mav.sendUE4PosScale(301,4038,0,[-132,43,0],[0,0,(1/2)*math.pi],[2,2,2])  
    #一楼桌子2 id=302
    mav.sendUE4PosScale(302,4038,0,[-132,63,0],[0,0,(1/2)*math.pi],[2,2,2])

    #一楼椅子1 id=303
    mav.sendUE4PosScale(303,5032,0,[-130,43,0],[0,0,(1/2)*math.pi],[1.5,1.5,1.5]) 
    #一楼椅子2 id=304
    mav.sendUE4PosScale(304,5032,0,[-129.5,63,0],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])



    #一楼窗户
    #一楼窗户1 关闭 （6孔窗）id=309
    mav.sendUE4PosScale(309,7039,0,[-128.5,47.77,-1.52],[0,0,(1/2)*math.pi],[2.2,1.35,2.33])
    #一楼窗户2 关闭 （6孔窗）id=310
    mav.sendUE4PosScale(310,7039,0,[-128.5,65.81,-1.52],[0,0,(1/2)*math.pi],[2.2,1.35,2.33])
    #一楼窗户3 关闭 （单孔窗）id=311
    mav.sendUE4PosScale(311,5039,0,[-144.6,69.79,-1.75],[0,0,(1/2)*math.pi],[1.85,1,2.6])
    #一楼窗户4 打开（6孔窗） id=312
    mav.sendUE4PosScale(312,5039,0,[-144.6,51.78,-1.52],[0,0,(1/2)*math.pi],[2.2,1.35,2.3]) 
    #一楼窗户5 打开（单孔窗） id=313
    mav.sendUE4PosScale(313,3039,0,[-134.5,38.85,-1.54],[0,0,(1)*math.pi],[2.2,1.3,2.7])
    #mav.sendUE4PosScale(313,7039,0,[-134.6,38.8,-1.52],[0,0,(1)*math.pi],[1.9,1.3,2.35])

    #一楼门 
    #一楼门2 打开的门 id=315
    mav.sendUE4PosScale(315,41,0,[-144.6,42.5,-0.9],[0,0,(3/4)*math.pi],[0.95,0.95,1.3]) 
    #打开门的门框
    mav.sendUE4PosScale(360,39,0,[-144.63,42.96,-1.4],[0,0,(1/2)*math.pi],[1.5,1.0,4.4])
    #移动门的门框
    mav.sendUE4PosScale(361,39,0,[-144.61,61.7,-1.35],[0,0,(1/2)*math.pi],[3,1.0,4.5])
    #二楼场景

    #二楼桌子

    #二楼桌子1 id=316
    mav.sendUE4PosScale(316,2038,0,[-138,46,-5],[0,0,(0)*math.pi],[2,3.8,1])
    #二楼桌子2 id=317
    mav.sendUE4PosScale(317,4038,0,[-139,69,-5],[0,0,(1/2)*math.pi],[2,2,2])

    #二楼椅子1 id=318
    mav.sendUE4PosScale(318,5032,0,[-136,42+1,-5],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子2 id=319
    mav.sendUE4PosScale(319,5032,0,[-136,44+1,-5],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子3 id=320
    mav.sendUE4PosScale(320,5032,0,[-136,46+1,-5],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子4 id=321
    mav.sendUE4PosScale(321,5032,0,[-136,48+1,-5],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子5 id=322
    mav.sendUE4PosScale(322,5032,0,[-140,42+1,-5],[0,0,(-1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子6 id=323
    mav.sendUE4PosScale(323,5032,0,[-140,44+1,-5],[0,0,(-1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子7 id=324
    mav.sendUE4PosScale(324,5032,0,[-140,46+1,-5],[0,0,(-1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子8 id=325
    mav.sendUE4PosScale(325,5032,0,[-140,48+1,-5],[0,0,(-1/2)*math.pi],[1.5,1.5,1.5])
    #二楼椅子9 id=326
    mav.sendUE4PosScale(326,5032,0,[-138,40.5,-5],[0,0,(0)*math.pi],[1.5,1.5,1.5])
    #二楼椅子10 id=327
    mav.sendUE4PosScale(327,5032,0,[-138,52,-5],[0,0,(1)*math.pi],[1.5,1.5,1.5])
    #二楼椅子11 id=328
    mav.sendUE4PosScale(328,4032,0,[-141,69,-5],[0,0,(0)*math.pi],[1.5,1.5,1.5])

    #二楼人物
    #二楼人物2 id=331
    mav.sendUE4PosScale(331,30,0,[-141+0.5,69,-5],[0,0,(0)*math.pi],[1.5,1.5,1.5])
    #二楼人物2头套 id=332
    mav.sendUE4PosScale(332,152,0,[-141+0.6,69,-5-2.6],[0,0,math.pi],[0.11,0.11,0.11])
    #二楼人物3 id=333
    mav.sendUE4PosScale(333,1030,0,[-141+4,69,-5],[0,0,(1)*math.pi],[1.5,1.5,1.5])
    #二楼人物3头套 id=334
    mav.sendUE4PosScale(334,1152,0,[-141+3.9,69,-5-2.4],[0,0,(1)*math.pi],[0.11,0.11,0.11])


    #二楼窗户

    #二楼窗户1 id=337
    mav.sendUE4PosScale(337,3039,0,[-128.6,47.78,-6.52],[0,0,(1/2)*math.pi],[2.8,1.3,2.9])

    #二楼窗户2 关闭单孔窗id=338
    mav.sendUE4PosScale(338,3039,0,[-128.6,65.79,-6.52],[0,0,(1/2)*math.pi],[2.8,1.3,2.9])
    #二楼窗户3 关闭单孔窗id=339
    mav.sendUE4PosScale(339,3039,0,[-128.6,76.8,-6.52],[0,0,(1/2)*math.pi],[2.8,1.3,2.9])
    #二楼窗户4 关闭单孔窗id=340
    mav.sendUE4PosScale(340,3039,0,[-130.53,78.8,-6.55],[0,0,(0)*math.pi],[2.6,1.25,2.9])
    #二楼窗户5 关闭单孔窗id=341
    mav.sendUE4PosScale(341,3039,0,[-138.51,78.8,-6.55],[0,0,(0)*math.pi],[2.6,1.25,2.9])
    #二楼窗户6 打开6孔窗id=342
    mav.sendUE4PosScale(342,5039,0,[-144.61,68.77,-6.48],[0,0,(1/2)*math.pi],[1.9,1.3,2.5])
    #二楼窗户7 打开单孔窗id=343
    mav.sendUE4PosScale(343,39,0,[-144.61,56.45,-6.48],[0,0,(1/2)*math.pi],[2,1.3,2.5])
    #二楼窗户8 打开6孔窗id=344
    mav.sendUE4PosScale(344,5039,0,[-144.61,44.5,-6.48],[0,0,(1/2)*math.pi],[2,1.3,2.5])
    #二楼窗户9 关闭单孔窗id=345
    mav.sendUE4PosScale(345,3039,0,[-130.53,38.85,-6.52],[0,0,(0)*math.pi],[2.5,1.3,2.9])
    #二楼窗户10 关闭单孔窗id=346
    mav.sendUE4PosScale(346,3039,0,[-138.50,38.85,-6.52],[0,0,(0)*math.pi],[2.5,1.3,2.9])

    print("attack_res1:",AttackRes)
    
    #一楼人物1 走动的人 id=305
    socketRevAttack=SocketRevAttack()

    t1=Thread(target=recive_attack_res)
    t2=Thread(target=move_man2)


    # #一楼人物1 id=305
    # mav.sendUE4PosScale(305,9030,0,[-137,50,-0.2],[0,0,(0)*math.pi],[1.5,1.5,1.5])
    # #一楼人物1头套 id=306
    # mav.sendUE4PosScale(306,152,0,[-137,50,-2.8],[0,0,math.pi],[0.11,0.11,0.11])   

    #二楼移动的人

    t3=Thread(target=move_man)



    #二楼移动的人
    t4=Thread(target=move_man3)


    # #二楼人物1 id=329
    # mav.sendUE4PosScale(329,6030,0,[-138,41,-5],[0,0,(1/2)*math.pi],[1.5,1.5,1.5])
    # #二楼人物1头套 id=330
    # mav.sendUE4PosScale(330,152,0,[-138,41,-5-2.4],[0,0,math.pi],[0.11,0.11,0.11])

    #二楼移动的人
    t5=Thread(target=move_door1)

    #一楼移动的门
    t6=Thread(target=move_door2)

    #二楼移动的门
    t7=Thread(target=move_door3)

    t8=Thread(target=f1_man2)
    
    t9=Thread(target=f2_man4)
    t10=Thread(target=move_door4)
    #二楼门 
    #二楼门2 打开的门 id=349
    mav.sendUE4PosScale(349,41,0,[-132.6,46.88,-5.1],[0,0,(3/4)*math.pi],[1,1,0.95]) 
    #二楼门2 打开的门 id=350
    mav.sendUE4PosScale(350,41,0,[-132.6,48.6,-5.1],[0,0,(1/4)*math.pi],[1,1,0.95])
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()
    t6.start()
    t7.start()
    t8.start()
    t9.start()
    t10.start()
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()
    t6.join()
    t7.join()
    t8.join()
    t9.join()
    t10.join()
    while 1:
            time.sleep(0.05)



    # #桌子 
    # mav.sendUE4PosScale(98,38,0,[-138,50-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(97,1038,0,[-138,52-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(96,2038,0,[-138,54-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(95,3018,0,[-138,56-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(94,4038,0,[-138,58-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(93,5038,0,[-138,60-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(92,6038,0,[-138,62-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # #window 
    # mav.sendUE4PosScale(91,39,0,[-136,50-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(90,1039,0,[-136,52-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(89,2039,0,[-136,54-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(88,3039,0,[-136,56-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(87,4039,0,[-136,58-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(86,5039,0,[-136,60-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(85,6039,0,[-136,62-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # mav.sendUE4PosScale(84,7039,0,[-136,64-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # mav.sendUE4PosScale(83,8039,0,[-136,66-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # #椅子
    # mav.sendUE4PosScale(82,32,0,[-134,50-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(81,1032,0,[-134,52-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(80,8032,0,[-134,54-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(79,3032,0,[-134,56-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(78,4032,0,[-134,58-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(77,5032,0,[-134,60-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(76,6032,0,[-134,62-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # mav.sendUE4PosScale(75,7032,0,[-134,64-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # #人物


    # #Dennis
    # mav.sendUE4PosScale(74,30,0,[-134,50-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5]) 
    # #Carla
    # mav.sendUE4PosScale(73,7030,0,[-134,52-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # # mav.sendUE4PosScale(72,5530,0,[-134,54,-5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(71,6030,0,[-134,56-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # #Claudia
    # mav.sendUE4PosScale(70,9030,0,[-134,58-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # #mav.sendUE4PosScale(69,7530,0,[-134,60,-5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(68,8030,0,[-134,62-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # #Eric
    # mav.sendUE4PosScale(67,9030,0,[-134,64-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # mav.sendUE4PosScale(66,10030,0,[-134,66-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])
    # mav.sendUE4PosScale(65,11030,0,[-134,68-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])


    # #门
    # mav.sendUE4PosScale(64,41,0,[-134-10,50-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(63,1041,0,[-134-10,52-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(62,2041,0,[-134-10,54-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(61,3041,0,[-134-10,56-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  
    # mav.sendUE4PosScale(60,4041,0,[-134-10,58-38,-5+5],[0,0,(0)*math.pi],[0.5,0.5,0.5])  







    # # RflyChange3DModel command followed by vehicle ID + desired style
    # #Send a message to make CopterID=100 (the character just created) in all scenes, here style=12 represents a walking person
    # mav.sendUE4Cmd(b'RflyChange3DModel 100 12') 
    # time.sleep(0.5)


    # # Command RflyChangeViewKeyCmd means to simulate the shortcut key pressed in RflySim3D, shortcut key B 1 means to switch the focus to the object with CopterID=1
    # # Here is set to send to window 0, other windows do not send
    # mav.sendUE4Cmd(b'RflyChangeViewKeyCmd B 1',0)
    # time.sleep(0.5)  


    # # Shortcut key V 1 means to switch to the 1st onboard camera (front camera)
    # mav.sendUE4Cmd(b'RflyChangeViewKeyCmd V 1',0)
    # time.sleep(0.5)  


    # # RflyCameraPosAng x y z roll pith yaw 
    # # Set the position of the camera relative to the center of the body, the default is 0
    # # Here set the position of the front camera to [0.1 -0.25 0]
    # mav.sendUE4Cmd(b'RflyCameraPosAng 0.1 0 0',0)
    # time.sleep(0.5)


    # # r.setres 720x405w is a built-in command of UE4, which means to switch the resolution to 720x405
    # mav.sendUE4Cmd(b'r.setres 720x405w',0)
    # time.sleep(0.5)




    # # Send a shortcut command to window 1 to switch the focus to vehilce 1
    # mav.sendUE4Cmd(b'RflyChangeViewKeyCmd B 1',1)
    # time.sleep(0.5)  


    # # Send a shortcut key control command to window 0, N 1 shortcut key means to switch the perspective to the ground fixed perspective 1
    # mav.sendUE4Cmd(b'RflyChangeViewKeyCmd N 1',1)
    # time.sleep(0.5)  

    # # Set the current camera Field of View (FOV) to 90 degrees (the default value is 90 degrees in RflySim3D), the range of FOV is 0 to 180 degrees
    # mav.sendUE4Cmd(b'RflyCameraFovDegrees 90',1)
    # time.sleep(0.5) 

    # # Set the current camera position here as [-2 0 -9.7]
    # mav.sendUE4Cmd(b'RflyCameraPosAng -2 0 -9.7',1)
    # time.sleep(0.5)


    # #Turn on MAVLink to monitor CopterSim data and update it in real time. 
    # mav.InitMavLoop()
    # time.sleep(0.5)


    # #Display Position information received from CopterSim
    # print(mav.uavPosNED)


    # #Turn on Offboard mode
    # mav.initOffboard()
    # # Send the desired position signal, fly to the target point 0,0, -1.7 position, the yaw angle is 0
    # mav.SendPosNED(0, 0, -1.7, 0) 
    # print("Send target Pos")
    # time.sleep(0.5)
    # #Send arm command to arm the drone
    # mav.SendMavArm(True) 
    # print("Send Arm Command")

    # time.sleep(10)

    # # Send the desired speed signal, 0.2m/s downwards, the z-axis downward is positive
    # mav.SendVelNED(0, 0, 0.2, 0) 
    # print("Send Velocity Speed")

    # time.sleep(10)

    # #Exit Offboard control mode
    # print("Send offboard stop")
    # mav.endOffboard()
    # time.sleep(1)

    # #Exit MAVLink data receiving mode
    # print("Send Mavlink stop")
    # mav.stopRun()
    # time.sleep(1)
    # #while True:
    # #    print(mav.uavPosNED)
    # #    time.sleep(2)
