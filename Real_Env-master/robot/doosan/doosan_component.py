import os
import sys
import math
from scipy.spatial.transform import Rotation as R
import numpy as np
#import torch
from .doosan_api import DoosanAPI
from base import BaseRobotComponent, RobotControlMode
#from .ee_force_compensation import CompensationNet
from controller import *

class DoosanComponent(BaseRobotComponent):

    def __init__(
            self,
            robot_ip,
            controller_type="eac_pos",
            frequency=200,
            max_vel=30,
            max_acc=50,
    ):
        super().__init__(urdf="doosan", controller_type=controller_type, frequency=frequency)
        self.api = DoosanAPI(ip=robot_ip, dt=self.dt, max_vel=max_vel, max_acc=max_acc)

        # self.compensation_net = CompensationNet()
        # self.compensation_net.load_state_dict(
        #     torch.load(os.path.join(os.path.dirname(__file__), "compensation.pth"), map_location=torch.device("cpu"))
        # )
        # self.compensation_net.eval()

        def movej(jpos):
            return self.api.movej(np.asarray(jpos) * 180 / math.pi)
        self.register_control_function("joint_pos", movej)

        def joint_pos_rt(jpos):
            print("3")
            return self.api.amovej(np.asarray(jpos) * 180 / math.pi, max_vel, max_acc, target_time=1.0)
        self.register_control_function("joint_pos_rt", joint_pos_rt)

        def eef_speed_rt(eef_vel):
            print("4444444444")
            #print("eef_vel1", eef_vel)
            #dt = 1.0 / freq
            eef_vel[:3] = eef_vel[:3] * 1000
            eef_vel[3:] = eef_vel[3:] * 180 / math.pi
            #print("eef_vel2", eef_vel)
            target_acc = np.array([0.1, 0.1, 0.1] + [max_acc] * 3)
            #print("target_acc", target_acc)
            return self.api.speedl_rt(target_vel=eef_vel, target_acc=target_acc, target_time=self.dt)

        def eef_servol(self, eef_vel):
            print("777777777")
            print("eef_vel1", eef_vel)
            # dt = 1.0 / freq
            eef_vel[:3] = eef_vel[:3] * 100
            eef_vel[3:] = eef_vel[3:] * 180 / math.pi
            print("eef_vel2", eef_vel)
            target_acc = np.array([0, 0, 0, 0, 0, 0])
            # print("target_acc", target_acc)
            return self.api.speedl_rt(target_vel=eef_vel, target_acc=target_acc, target_time=1 / 500)

        self.register_control_function("eef_speed_rt", eef_speed_rt)
        self.register_control_function("eef_servol", eef_servol)
        self.register_control_function("joint_torque_rt", self.api.torque_rt)

        self.robot_state, self.robot_mode = None, None

    def eef_servol(self,eef_pos):
            #print("777777777")
            print("eef_vel1", eef_pos)
            #dt = 1.0 / freq
            eef_pos[:3] = eef_pos[:3]
            eef_pos[3:] = eef_pos[3:] * 180 / math.pi
            print("eef_vel2", eef_pos)
            target_vel = np.array([0, 0, 0, 0, 0, 0])
            target_acc = np.array([0,0,0,0,0,0])
            #print("target_acc", target_acc)
            return self.api.servol(eef_pos,target_vel=target_vel, target_acc=target_acc, target_time=1/500)

    def _switch_mode(self, control_mode: RobotControlMode):
        # print(f"old control_mode: {self.control_mode}, new control_mode: {control_mode}")
        if self.control_mode != RobotControlMode.NORMAL.value and control_mode == RobotControlMode.NORMAL.value:
            self.api.switch_mode("normal")
        else:
            self.api.switch_mode("rt")
            print("set RobotControlMode=rt")



    def movej(self,jpos):
        return self.api.movej(np.asarray(jpos) * 180 / math.pi)
    def get_control_dict(self):
        with self.lock:
            rt_output_data_list = self.api.read_data_rt()
            time_stamp = rt_output_data_list.time_stamp
            actual_tcp_position = np.array(rt_output_data_list.actual_tcp_position, dtype=float)
            lin_pos = actual_tcp_position[:3] * 0.001
            quat_ori = R.from_euler("ZYZ", actual_tcp_position[3:], degrees=True).as_quat()
            eef_pos_quat = np.concatenate([lin_pos, quat_ori])#当前末端的位姿

            actual_tcp_velocity = np.array(rt_output_data_list.actual_tcp_velocity, dtype=float)
            lin_vel = actual_tcp_velocity[:3] * 0.001
            ang_vel = actual_tcp_velocity[3:] * math.pi / 180
            eef_vel = np.concatenate([lin_vel, ang_vel])#当前末端的速度

            actual_joint_position = np.array(rt_output_data_list.actual_joint_position, dtype=float)
            joint_pos = actual_joint_position * math.pi / 180 #当前的关节角度

            actual_joint_velocity = np.array(rt_output_data_list.actual_joint_velocity, dtype=float)
            joint_vel = actual_joint_velocity * math.pi / 180 #当前的关节速度

            joint_torque = np.array(rt_output_data_list.actual_joint_torque, dtype=float) #当前的关节力矩

            joint_ex_torque = np.array(rt_output_data_list.external_joint_torque, dtype=float) #当前外力产生的关节力矩

            eef_ex_force = np.array(rt_output_data_list.external_tcp_force, dtype=float)  #外力产生的末端力
            '''
            if hasattr(self, "compensation_net"): #https://blog.csdn.net/lsoxvxe/article/details/131651168
                with torch.no_grad():
                    compensation = self.compensation_net(eef_pos_quat)
                print(f"ex_force: {eef_ex_force}, compensation: {compensation}")
                eef_ex_force = eef_ex_force - compensation   #这是通过网络对力进行补偿，上面注释了所以不执行此操作
                '''
            control_dict = {
                "position_limits": self.position_limits,
                "eef_pos_quat": eef_pos_quat,  # shape of (3 + 4), the (lin_pos: 3, quat_ori: 4) state of the eef body
                "eef_vel": eef_vel,  # shape of (6), the (lin_vel: 3, ang_vel: 3) state of the eef body
                "joint_pos": joint_pos,  # shape of (n_dof), the (joint_pos) state of the eef body
                "joint_vel": joint_vel,  # shape of (n_dof), the (joint_vel) state of the eef body
                "joint_torque": joint_torque,  # shape of (n_dof), the (joint_torque) state of the eef body
                "joint_ex_torque": joint_ex_torque,  # shape of (n_dof), the (joint_torque) state of the eef body
                "eef_ex_force": eef_ex_force,  # shape of (n_dof), the (joint_torque) state of the eef body
                "timestamp": time_stamp,
            }
            # print(f"------------------------------------------------------")
            # print(f'eef_pos_quat:           {control_dict["eef_pos_quat"]}')
            # print(f'eef_vel:                {control_dict["eef_vel"]}')
            # print(f'joint_torque:           {control_dict["joint_torque"]}')
            # print(f'joint_ex_torque:        {control_dict["joint_ex_torque"]}')
            # print(f'eef_ex_force:          {control_dict["eef_ex_force"]}')
            return control_dict
            #返回上面的信息，以字典的形式
    @property
    def rest_qpos(self):
        return np.array([175, -17.5, -84.15, -1.2, -78.28, -92.61]) * 3.14 / 180

    @property
    def position_limits(self):  # 278.6
        return np.array([[350, -220, 11], [600, 220, 240]]) * 0.001

    @property
    def n_dof(self):
        return 6

