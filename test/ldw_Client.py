#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Date    : 2021-05-13 16:15:29
# @Author  : BrightSoul (653538096@qq.com)


import os


import socket
import time
import re

import signal
import subprocess




BUFSIZE = 1024
ip='192.168.1.192'
port=63000

# rflyCMD = "C:/PX4PSP/RflySimAPIs/_High-speed/client_ue4_SITL.bat"
clientCMD = "/home/nvidia/Programing/lwd_ws/src/attack.sh"
task_time = 80

# clientCMD = "roslaunch attack data_saver.launch"
# clientCMD = "/home/nvidia/Programing/lwd_ws/src/test.sh"
# task_time = 10


clientCMD_cwd = "/home/nvidia/Programing/lwd_ws"
# pythonCMD = "cmd /k C:\\PX4PSP\\Python38\\python.exe client_grass.py"




            

class RflyClient():
    def __init__(self,ip='127.0.0.1',port=63000):
        self.client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.client.connect((ip,port))
        self.client.setblocking(False)
        

        self.clientTask = None


        self.D_cmd = {"strat":b"cmd01","stop":b"cmd02","end":b"cmdff"}
        print(f"Try to connect Server {ip} on {port}")

    def connect_server(self):
        while True:
            message = b"Hello Server"
            self.client.sendall(message)
            time.sleep(1)
            try:
                data,address = self.client.recvfrom(BUFSIZE)
            except Exception as e:
                print("no connection")
                continue

            if data == b"Server Received":
                print("Server Received")
                break

    def run(self):
        while True:
            control_word, _ = self.server.recvfrom(BUFSIZE)
            if control_word == self.D_cmd["strat"]:
                self.start()
            elif control_word == self.D_cmd["stop"]:
                self.stop()
            elif control_word == self.D_cmd["end"]:
                self.end()
                break

    def get_killpid(self):
        find_port = 'netstat -aon|findstr "63000"'
        result = os.popen(find_port)
        pid_this= (result.read()[-7:]).strip()
        
        find_killport = 'tasklist|find /i "python.exe"' 
        result = os.popen(find_killport)
        L_pid = re.findall(r"python.exe.*?(\d+)",result.read())
        L_pid.remove(pid_this)
        return L_pid
    
    def start(self):
        # preexec_fn=os.setpgrp设置这个参数也蛮重要的
        self.clientTask = subprocess.Popen(clientCMD,stdin=subprocess.PIPE,cwd=clientCMD_cwd,preexec_fn=os.setpgrp)
        # self.clientTask = subprocess.Popen(clientCMD,stdin=subprocess.PIPE,cwd=clientCMD_cwd)
    
    def stop(self):
        print("task stop")
        # print(self.clientTask.pid)
        # pgid = os.getpgid(self.clientTask.pid)
        # os.killpg(pgid, signal.SIGKILL)
        # self.clientTask.send_signal(signal.SIGINT)
        # out,err = self.clientTask.communicate(input=b"^C")
        
        # 杀死self.clientTask.pid进程下的子进程
        os.kill(-self.clientTask.pid, signal.SIGINT)
        
        # os.kill(self.clientTask.pid, signal.CTRL_C_EVENT)
        # time.sleep(1)
        # os.system("kill -9 `ps -e | grep ros | awk '{print $1}'`")
    
    def end(self):
        print("task end")
        self.clientTask.terminate()
        self.client.close()



def main():
    RC = RflyClient(ip,port)
    RC.wait_client()
    RC.run()

def test_client():
    RC = RflyClient(ip,port)
    RC.start()
    time.sleep(task_time)
    RC.stop()
    time.sleep(5)
    RC.end()

if __name__ == '__main__':
    # main()
    test_client()
