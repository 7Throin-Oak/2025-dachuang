import os
import time
import numpy as np
from robot.doosan.doosan_component import DoosanComponent
from base import Command, RobotControlMode
#from data_utils import DynamicsDataset
from controller import *

GLOBAL_PATH = os.path.dirname(__file__)

#对输入的jpos数组进行限位：np.clip(a, a_min, a_max）
def joint_pos_clip(jpos):
    return np.clip(
        jpos, np.array([-3.1, -1.6, -2.3, -3.14, -2.3, -3.1]), np.array([3.1, 1.6, 2.3, 3.1, 2.3, 3.1])
    )



def replay(robot_ip, skip, freq=500):
    dt = 1.0 / freq
    #teach_dataset = DynamicsDataset(name="doosan_teach")

    doosan_robot = DoosanComponent(
        controller_type="eac_pos",
        robot_ip=robot_ip,
    )
    doosan_robot.start()
    time.sleep(1)
    rest_qpos = doosan_robot.rest_qpos
    print("start to replay...")
    print("rest_qpos=",rest_qpos)
    start = 0
    doosan_robot.control(cmd=Command.MOVEJ.value, action=rest_qpos)#将机械臂移动到初始采集点的位置
    time.sleep(3)
    doosan_robot.busy_event.wait()
    # time.sleep(100)
    offset = np.zeros(6)
    while 1:

        #print("111111111")
        q=np.asarray([0,0,0])
        doosan_robot.control(cmd=Command.SET_GOAL.value, action=q, duration=1 * dt)
        doosan_robot.busy_event.wait()
        time.sleep(dt)


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    robot_ip = "192.168.5.110"
    # teach(robot_ip=robot_ip, freq=20)
    replay(robot_ip=robot_ip, skip=1, freq=200)
