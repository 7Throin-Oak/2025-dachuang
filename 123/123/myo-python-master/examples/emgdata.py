#!/usr/bin/python
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
from FeatureSpace import FeatureSpace
import joblib
import platform
import os
from offlineClf import myModel
#import rtde_control

import sys
sys.path.append('..')



class EmgCollector(myo.DeviceListener):
  """
  Collects EMG data in a queue with *n* maximum number of elements.
  """

  def __init__(self, n):
    self.n = n
    self.lock = Lock()
    self.emg_data_queue = deque(maxlen=n)
    #self.emgData = []  # 用于存储肌电数据
    self.emgDataCount = 0
    #self.emgDataLenth = 24  # 每次采集多少个数据做一次特征提取和动作的识别
    self.emgDict = dict()
    '''model'''
    self.modelFilePath = 'data/model/test728-model.pkl'  # 分类模型保存的路径
    self.numberVoter = 3  # 投票成员的个数

  def start(self):


    self.getSystermType()
    # self.setFilePath() #设置路径
    self.loadModel()  # 导入模型

  def get_emg_data(self):
    with self.lock:
      return list(self.emg_data_queue)

  # myo.DeviceListener

  def on_connected(self, event):
    event.device.stream_emg(True)

  def on_emg(self, event):
    with self.lock:
      #self.emgData = list(self.emgData)
      #self.emgData.append(event.emg)
      self.emgDataCount += 1
      self.emg_data_queue.append(event.emg)
      # try:
      #   csv_writer.writerow(event.emg)
      #   #csv_writer.writerow(str(time.time()))
      #   # csv_writer.writerow((event.timestamp,event.emg))
      #   #self.calculate_activation(event.emg,1,-1.5)
      #   #print(self.activition)
      # except Exception as e:
      #   raise e
  def onlineClf(self):

    if self.emgDataCount > self.model.mWinWidth + self.numberVoter - 1:  # 投票数为其加1
      pre = time.time()
      #self.emgDataCount = 0
      self.emgData = self.get_emg_data()
      self.emgData = np.array(self.emgData).T
      self.emgDict['one'] = self.emgData

      self.sample = FeatureSpace(rawDict=self.emgDict,
                                 moveNames=['one', ],  # 动作类别
                                 ChList=[0, 1, 2, 3, 4, 5, 6, 7],  # 传感器的通道数目
                                 features={'Names': self.model.mFeatureList,  # 定义的特征
                                           'LW': self.model.mWinWidth,  # 定义的窗宽
                                           'LI': self.model.mSlidingLen},  # 定义的窗滑动的步长
                                 one_hot=False,
                                 trainPercent=[1, 0, 0]  # 是否进行onehot处理
                                 )
      self.getTrainData()
      actionList = self.model.mModel.predict(self.trainX)
      label=self.getTheAction(actionList)
      #print(label)
      self.emgDict.clear()
      if time.time() - pre>0.02:
        print(time.time() - pre)
      return label
    else:
      time.sleep(0.001)


  # The feature space is divided into data sets
  def getTrainData(self):

    nDimensions = self.sample.trainImageX.shape[1]
    # 训练集
    self.trainX = np.reshape(self.sample.trainImageX, (-1, nDimensions))
    self.trainY = np.squeeze(self.sample.trainImageY)
    # 测试集
    self.testX = np.reshape(self.sample.testImageX, (-1, nDimensions))
    self.testY = np.squeeze(self.sample.testImageY)
    # 评估集
    self.validateX = np.reshape(self.sample.validateImageX, (-1, nDimensions))
    self.validateY = np.squeeze(self.sample.validateImageY)

  '''导入已经保存的模型'''

  def loadModel(self):

    print("path:", self.modelFilePath)
    self.model = joblib.load(self.modelFilePath)
    self.actionNames = self.model.actionNames

  '''设置模型文件的名称'''

  def setModelFileName(self, fileName):

    self.modelFileName = fileName

  def getSystermType(self):

    self.systermType = platform.system()

  '''设置模型文件的路径'''

  def setFilePath(self):

    currentFilePath = os.getcwd()
    if self.systermType == "Windows":
      self.modelFilePath = currentFilePath + '//data//model//'
      self.filePathName = currentFilePath + '//data//filePathFile.txt'
    elif self.systermType == "Linux":
      self.modelFilePath = currentFilePath + '/data/model/'
      self.filePathName = currentFilePath + '/data/filePathFile.txt'
    with open(self.filePathName, "r") as fp:
      self.modelFileName = str(fp.readlines()[1]) + "-model.pkl"
      fp.close()
    # 判断文件夹是否存在，不存在则创建
    if os.path.exists(self.modelFilePath) == True:
      self.modelFileExitFlag = True
    else:
      print("The model is not saved, please save the model before using the model!!")

  # 投票函数
  def getTheAction(self, actionList):
    #an=5*actionList.count('0')+10*actionList.count('1')+15*actionList.count('2')+20*actionList.count('3')+25*actionList.count('4')+30*actionList.count('5')
    tempData = np.array(actionList)
    counts = np.bincount(tempData)
    actionNumber = np.argmax(counts)
    return self.actionNames[actionNumber]  # 返回定义的动作类别字符串



class Plot(object):

  def __init__(self, listener):
    self.n = listener.n
    self.listener = listener
    self.fig = plt.figure()
    self.axes = [self.fig.add_subplot('81' + str(i)) for i in range(1, 9)]
    [(ax.set_ylim([-100, 100])) for ax in self.axes]
    self.graphs = [ax.plot(np.arange(self.n), np.zeros(self.n))[0] for ax in self.axes]
    plt.ion()

  def update_plot(self):
    emg_data = self.listener.get_emg_data()
    emg_data = np.array([x[1] for x in emg_data]).T
    for g, data in zip(self.graphs, emg_data):
      if len(data) < self.n:
        # Fill the left side with zeroes.
        data = np.concatenate([np.zeros(self.n - len(data)), data])
      g.set_ydata(data)
    plt.draw()

  def main(self):
    pret=time.time()
    while True:
      self.update_plot()
      if time.time()-pret>5 and time.time()-pret<8:
        print('start')
      if time.time()-pret>35:
        print('stop')
      plt.pause(1.0 / 30)




def main():
  pass




if __name__ == '__main__':
  myo.init()
  hub = myo.Hub()
  listener = EmgCollector(21)
  listener.start()

  # rtde_c = rtde_control.RTDEControlInterface("192.168.3.101")
  # acceleration = 0.1
  # pose=[-0.13636229549857265, 0.6168228123530642, 0.3011648342963643, -1.6050510932013433, 0.08159716000935488,
  #  -0.037070111577062324]
  # rtde_c.moveL(pose, 0.2, 0.3)
  # lastlabel='rest'



  with hub.run_in_background(listener.on_event):
    while True:
      label=listener.onlineClf()
      print(label)
      # if label!=lastlabel:
      #   print(label)
      #   if label=='left':
      #     speed = [0.03, 0.0, 0.0, 0.0, 0.0, 0.0]
      #     rtde_c.speedL(speed, acceleration)
      #   elif label=='right':
      #     speed = [-0.03, 0.0, 0.0, 0.0, 0.0, 0.0]
      #     rtde_c.speedL(speed, acceleration)
      #   # elif label=='up':
      #   #   speed = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]
      #   #   rtde_c.speedL(speed, acceleration)
      #   # elif label=='down':
      #   #   speed = [0.0, -0.05, 0.0, 0.0, 0.0, 0.0]
      #   #   rtde_c.speedL(speed, acceleration)
      #   elif label=='fist':
      #     speed = [0.0, 0.0, -0.03, 0.0, 0.0, 0.0]
      #     rtde_c.speedL(speed, acceleration,1.0)
      #   elif label=='grasp':
      #     speed = [0.0, 0.0, 0.03, 0.0, 0.0, 0.0]
      #     rtde_c.speedL(speed, acceleration,1.0)
      #   else:
      #     rtde_c.speedStop()
      #   lastlabel=label
      #print('0')