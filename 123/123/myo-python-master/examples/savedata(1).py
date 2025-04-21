#!/usr/bin/python
# -*-coding: utf-8 -*-
# 单手环测试
#import rtde.rtde as rtde
from matplotlib import pyplot as plt
from collections import deque
from threading import Lock, Thread
from datetime import datetime
import myo
import numpy as np
import csv
import time
from scipy import signal
import socket
#import struct
import rtde_receive
import rtde_control
import  random
#from offlineTest import myModel


class EmgCollector(myo.DeviceListener):
    """
    Collects EMG data in a queue with *n* maximum number of elements.
    """

    def __init__(self, n, filename):

        self.n = n
        self.lock = Lock()
        self.filename=filename
        self.emg_data_queue = deque(maxlen=n)

    def get_emg_data(self):
        with self.lock:
            return list(self.emg_data_queue)

    # myo.DeviceListener

    def on_connected(self, event):
        event.device.stream_emg(True)

    def on_emg(self, event):
        with self.lock:
            global rtde_r
            global f
            #self.emg_data_queue.append((event.timestamp, event.emg))
            ls=event.emg
            ls.extend(rtde_r.getActualTCPForce())
            ls.extend(rtde_r.getActualTCPPose())
            ls.extend(rtde_r.getActualTCPSpeed())
            #print(actual_q)
            #ppre=time.time()
            #state = con.receive(True)
            #voltage = struct.unpack('!12d', state)
            #print(voltage)
            ls.append(event.timestamp)
            #print(time.time()-pre)
            #sk.close()

            # with open(self.filename, 'a+', newline='') as f:
            writer = csv.writer(f)
                # a=[str(i) for i in struct.unpack(fmt, data1)]
            writer.writerow(ls)  # 保存数据为csv格式
            #f.close()


if __name__ == '__main__':
    global rtde_r
    global f
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.3.101")
    rtde_c = rtde_control.RTDEControlInterface("192.168.3.101")

    task_frame = [0, 0, 0, 0, 0, 0]
    selection_vector = [1, 1, 0, 0, 0, 0]
    wrench_up = [0, 0, 0, 0, 0, 0]  # 修改
    force_type = 2
    limits = [5, 5, 5, 1, 1, 1]
    dt = 1.0 / 150  # 2ms
    # joint_q = [-1.54, -1.83, -2.28, -0.59, 1.60, 0.023]
    joint_q = [-1.4352682272540491, -1.8628307781615199, -2.2286536693573, -0.6000073117068787, 1.582466959953308,
               0.14063385128974915]


    rtde_c.moveJ(joint_q)   # 机器人移动到初始位姿


    rtde_c.forceModeSetDamping(0.005)
    #forceModeSetGainScaling()
    rtde_c.forceMode(task_frame, selection_vector, wrench_up, force_type, limits)
    # con = rtde.RTDE('192.168.3.101', 30004)
    # con.connect()
    # output_names = ['actual_TCP_force','actual_TCP_pose']
    # output_types = ['VECTOR6D','VECTOR6D']
    # con.send_output_setup(output_names, output_types, frequency=200)
    # con.send_start()
    filename = 'data/emgData/5555.csv'

    f=open(filename, 'a+', newline='')
    writer = csv.writer(f)
    writer.writerow(['ch0', 'ch1', 'ch2', 'ch3', 'ch4', 'ch5', 'ch6', 'ch7','fx','fy','fz','frx','fry','frz','x','y','z','rx','ry','rz'])  # 为肌电数据设置列索引
    #f.close()
    myo.init()
    hub = myo.Hub()
    listener = EmgCollector(512,filename)
    with hub.run_in_background(listener.on_event):
        # Plot(listener).main()
        pret = time.time()
        while True:
            if time.time() - pret >= 5 and time.time() - pret <65:
                print('start')
                #a = random.choice([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
                #x2[0] = ox[a] + d
                #x2[1] = oy[a] + d
                #rtde_c.moveL(x2, 3, 3, True)
                #time.sleep(0.02)
                #rtde_c.moveL(x1, 3, 3, True)
                # rtde_c.moveL(x2)
                # time.sleep(1)
                # rtde_c.moveL(x1)
                # rtde_c.servoL(x2, velocity, acceleration, dt, lookahead_time, gain)
                # rtde_c.servoL(x1, velocity, acceleration, dt, lookahead_time, gain)
                #i = i + 1
                #time.sleep(0.02)
                #print(i)
            if time.time() - pret >=65:
                print('stop')
                rtde_c.forceModeStop()
                rtde_c.stopScript()
                break
            time.sleep(1)

