#!/usr/bin/python
#coding=utf-8
# -*-coding: utf-8 -*-

from matplotlib import pyplot as plt
from collections import deque
from threading import Lock, Thread
from datetime import datetime
import myo
import numpy as np
import csv
import time
from scipy import signal
import string

class EmgCollector(myo.DeviceListener):
  """
  Collects EMG data in a queue with *n* maximum number of elements.
  """
  def __init__(self, n):
    self.n = n
    self.lock = Lock()
    self.emg_data_queue = deque(maxlen=n)
    self.actionNum = 0  # 用于对动作类别进行计数
    self.curaction = 0
    self.actionList = ["0", "left", "0", "right", "0", "grasp", "0", "fist", "0", "rest", '0',"left", "0", "right", "0", "grasp", "0", "fist", "0", "rest", '0']  # 保存的动作列表

  def get_emg_data(self):
    with self.lock:
      return list(self.emg_data_queue)

  # myo.DeviceListener

  def on_connected(self, event):
    event.device.stream_emg(True)

  def on_emg(self, event):
    global filename
    with self.lock:
      self.emg_data_queue.append((event.timestamp, event.emg))
      ls = event.emg

      ls.append(event.timestamp)
      # print(time.time()-pre)
      # sk.close()

      # with open(self.filename, 'a+', newline='') as f:
      writer = csv.writer(f)
      # a=[str(i) for i in struct.unpack(fmt, data1)]
      writer.writerow(ls)

      # try:
      #   csv_writer.writerow(event.emg)
      #   #csv_writer.writerow(str(time.time()))
      #   # csv_writer.writerow((event.timestamp,event.emg))
      #   #self.calculate_activation(event.emg,1,-1.5)
      #   #print(self.activition)
      # except Exception as e:
      #   raise e




if __name__ == '__main__':
  global filename
  filename = 'test1.csv'
  dt=datetime.now()
  nowtime_str = dt.strftime('%y-%m-%d %I-%M-%S')  # 时间
  filename = nowtime_str + '_' + filename
  f = open(filename, 'a+', newline='')
  writer = csv.writer(f)
  writer.writerow(['ch0', 'ch1', 'ch2', 'ch3', 'ch4', 'ch5', 'ch6', 'ch7'])
  myo.init()
  hub = myo.Hub()
  listener = EmgCollector(512)
  with hub.run_in_background(listener.on_event):
      pret = time.time()
      while True:
        if time.time() - pret >=10:
          print('stop')
          break

