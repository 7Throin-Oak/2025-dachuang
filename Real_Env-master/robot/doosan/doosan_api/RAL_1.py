import csv
import math
import sys
import os
import time
import platform
from typing import List
import numpy as np
from robot.doosan.doosan_api.api_types import *
import ctypes
from scipy.spatial.transform import Rotation as R
from ati_axia80_ethernet_python import ForceTorqueSensorDriver
from matplotlib import pyplot as plt
from collections import deque
from threading import Lock, Thread
from datetime import datetime
import myo
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



class DoosanAPI:

    def __init__(self, ip, dt, max_vel=0.03, max_acc=20 * 3.14 / 180):
        self.ip = ip
        self.dt = dt
        #fpath = os.path.dirname(os.path.realpath(__file__))
        fpath = os.path.split(os.path.abspath(__file__))[0] + '\\'
        #print(fpath)
        if sys.platform.startswith("linux"):
            bit_size = platform.architecture()[0]

            if bit_size == '32bit':
                dll = CDLL(os.path.join(fpath, "dll/linux/32bits/doosan.so"))
            elif bit_size == '64bit':
                dll = CDLL(os.path.join(fpath, "dll/linux/64bits/doosan.so"))
            else:
                print("无法确定系统位数。")
                assert False

        elif sys.platform.startswith("win"):
            print(os.path.join(fpath, 'dll','win','doosan.dll'))
            os.environ['PATH'] += os.pathsep + 'dll\win\doosan.dll'
            dll = CDLL(os.path.join(fpath, 'dll','win','doosan.dll'))
            #dll = ctypes.CDLL("./doosan.dll")

        else:
            assert False, sys.platform

        self.dll = dll

        # self.on_rt_monitoring_data_callback = self.get_on_rt_monitoring_data_callback()
        self.on_log_alarm_callback = self.get_on_log_alarm_callback()
        # ################################ BASIC FUNCTION ##########################################################
        self.open_connection = self.dll.open_connection
        self.open_connection.argtypes, self.open_connection.restype = [c_char_p, c_int], c_bool

        self.close_connection = self.dll.close_connection
        self.close_connection.argtypes, self.close_connection.restype = [], c_bool

        self.get_robot_mode = self.dll.get_robot_mode
        self.get_robot_mode.argtypes, self.get_robot_mode.restype = [], ROBOT_MODE

        self.set_robot_mode = self.dll.set_robot_mode
        self.set_robot_mode.argtypes, self.set_robot_mode.restype = [ROBOT_MODE], c_bool
        self.SetRobotMode = self.dll.SetRobotMode
        self.SetRobotMode.argtypes, self.SetRobotMode.restype = [ROBOT_MODE], c_bool

        self.set_safe_stop_reset_type = self.dll.set_safe_stop_reset_type
        self.set_safe_stop_reset_type.argtypes, self.set_safe_stop_reset_type.restype = [SAFE_STOP_RESET_TYPE], \
                                                                                        c_bool
        self.SetSafeStopResetType = self.dll.SetSafeStopResetType
        self.SetSafeStopResetType.argtypes, self.SetSafeStopResetType.restype = [SAFE_STOP_RESET_TYPE], c_bool

        self.get_robot_state = self.dll.get_robot_state
        self.get_robot_state.argtypes, self.get_robot_state.restype = [], ROBOT_STATE
        self.GetRobotState = self.dll.GetRobotState
        self.GetRobotState.argtypes, self.GetRobotState.restype = [], ROBOT_STATE

        self.set_robot_system = self.dll.set_robot_system
        self.set_robot_system.argtypes, self.set_robot_system.restype = [ROBOT_SYSTEM], c_bool
        self.SetRobotSystem = self.dll.SetRobotSystem
        self.SetRobotSystem.argtypes, self.SetRobotSystem.restype = [ROBOT_SYSTEM], c_bool

        self.set_safety_mode = self.dll.set_safety_mode
        self.set_safety_mode.argtypes, self.set_safety_mode.restype = [SAFETY_MODE, SAFETY_MODE_EVENT], c_bool

        self.set_robot_control = self.dll.set_robot_control
        self.set_robot_control.argtypes, self.set_robot_control.restype = [ROBOT_CONTROL], c_bool
        self.SetRobotControl = self.dll.SetRobotControl
        self.SetRobotControl.argtypes, self.SetRobotControl.restype = [ROBOT_CONTROL], c_bool

        self.get_library_version = self.dll.get_library_version
        self.get_library_version.argtypes, self.get_library_version.restype = [], c_char_p

        self.check_motion = self.dll.check_motion
        self.check_motion.argtypes, self.check_motion.restype = [], c_int

        self.ManageAccessControl = self.dll.ManageAccessControl
        self.ManageAccessControl.argtypes, self.ManageAccessControl.restype = [MANAGE_ACCESS_CONTROL], c_bool

        self.set_digital_output = self.dll.set_digital_output
        self.set_digital_output.argtypes, self.set_digital_output.restype = \
            [GPIO_CTRLBOX_DIGITAL_INDEX, c_bool], c_bool

        self.servo_off = self.dll.servo_off
        self.servo_off.argtypes, self.servo_off.restype = [STOP_TYPE], c_bool

        self.change_collision_sensitivity = self.dll.change_collision_sensitivity
        self.change_collision_sensitivity.argtypes, self.change_collision_sensitivity.restype = \
            [c_float], c_bool

        self.release_compliance_ctrl = self.dll.release_compliance_ctrl
        self.release_compliance_ctrl.argtypes, self.release_compliance_ctrl.restype = [], c_bool
        # #################################### SINGLE CONTROL ##################################################
        self.move_home = self.dll.move_home
        self.move_home.argtypes, self.move_home.restype = [MOVE_HOME, c_int], c_bool

        self.stop = self.dll.stop
        self.stop.argtypes, self.stop.restype = [STOP_TYPE], c_bool

        self.movel = self.dll.movel
        self.movel.argtypes, self.movel.restype = [c_float * 6, c_float * 2, c_float * 2,
                                                   c_float, MOVE_MODE, MOVE_REFERENCE, c_float,
                                                   BLENDING_SPEED_TYPE], c_bool

        self.servoj = self.dll.servoj
        self.servoj.argtypes, self.servoj.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT,
                                                     c_float * NUM_JOINT, c_float], c_bool

        self.speedj = self.dll.speedj
        self.speedj.argtypes, self.speedj.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT, c_float], c_bool

        # ######################################## RT CONTROL ########################################################
        self.connect_rt_control = self.dll.connect_rt_control
        self.connect_rt_control.argtypes, self.connect_rt_control.restype = \
            [c_char_p, c_int], c_bool

        self.disconnect_rt_control = self.dll.disconnect_rt_control
        self.disconnect_rt_control.argtypes, self.disconnect_rt_control.restype = [], c_bool

        self.set_rt_control_input = self.dll.set_rt_control_input
        self.set_rt_control_input.argtypes, self.set_rt_control_input.restype = \
            [c_char_p, c_float, c_int], c_bool

        self.set_rt_control_output = self.dll.set_rt_control_output
        self.set_rt_control_output.argtypes, self.set_rt_control_output.restype = \
            [c_char_p, c_float, c_int], c_bool

        self.start_rt_control = self.dll.start_rt_control
        self.start_rt_control.argtypes, self.start_rt_control.restype = [], c_bool

        self.stop_rt_control = self.dll.stop_rt_control
        self.stop_rt_control.argtypes, self.stop_rt_control.restype = [], c_bool

        self.set_velj_rt = self.dll.set_velj_rt
        self.set_velj_rt.argtypes, self.set_velj_rt.restype = [c_float * NUM_JOINT], c_bool

        self.set_accj_rt = self.dll.set_accj_rt
        self.set_accj_rt.argtypes, self.set_accj_rt.restype = [c_float * NUM_JOINT], c_bool

        self.servoj_rt = self.dll.servoj_rt
        self.servoj_rt.argtypes, self.servoj_rt.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT,
                                                           c_float * NUM_JOINT, c_float], c_bool

        self.speedj_rt = self.dll.speedj_rt
        self.speedj_rt.argtypes, self.speedj_rt.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT,
                                                           c_float], c_bool

        self.servol_rt = self.dll.servol_rt
        self.servol_rt.argtypes, self.servol_rt.restype = [c_float * 6, c_float * 6,c_float * 6,
                                                           c_float], c_bool

        # ######################################## RT CONTROL ########################################################
        self.initialize(ip=ip, dt=dt, max_vel=max_vel, max_acc=max_acc)

    def get_on_rt_monitoring_data_callback(self):
        @CFUNCTYPE(None, POINTER(RT_OUTPUT_DATA_LIST))
        def on_rt_monitoring_data_callback(rt_output_data_list: POINTER(RT_OUTPUT_DATA_LIST)):
            self.rt_output_data_list = rt_output_data_list.contents
            # print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        return on_rt_monitoring_data_callback

    def get_on_log_alarm_callback(self):
        @CFUNCTYPE(None, POINTER(LOG_ALARM))
        def on_log_alarm_callback(log: POINTER(LOG_ALARM)):
            iLevel = log.contents._iLevel
            iGroup = log.contents._iGroup
            iIndex = log.contents._iIndex
            szParam = log.contents._szParam
            szParam = [string_at(szParam[i]).decode("utf-8") for i in range(3)]
            # warnings.warn(f"alarm -- iLevel: {iLevel}, iGroup: {iGroup}, iIndex: {iIndex}, szParam: {szParam}")

        return on_log_alarm_callback

    def initialize(self, ip, dt, max_vel=0.03, max_acc=20 * 3.14 / 180):
        assert self.open_connection(create_string_buffer(ip.encode("utf-8")), 12345), \
            f"fail to connect robot doosan with tcp: {ip}!"
        initialize = self.dll.initialize
        initialize.argtypes, initialize.restype = [], None
        initialize()

        set_on_rt_monitoring_data = self.dll.set_on_rt_monitoring_data
        set_on_rt_monitoring_data.argtypes, set_on_rt_monitoring_data.restype = [TOnRTMonitoringDataCB], None
        # set_on_rt_monitoring_data(self.on_rt_monitoring_data_callback)

        set_on_log_alarm = self.dll.set_on_log_alarm
        set_on_log_alarm.argtypes, set_on_log_alarm.restype = [TOnLogAlarmCB], None
        set_on_log_alarm(self.on_log_alarm_callback)

        assert self.set_robot_mode(ROBOT_MODE.ROBOT_MODE_MANUAL)
        assert self.set_robot_system(ROBOT_SYSTEM.ROBOT_SYSTEM_REAL)
        #tool = self.get_tool()
        #print("tool: ", tool)
        #if tool != "RG6":
            #print("add tool: ", self.add_tool(name="RG6", weight=1.25, cog=[0.0, 0, 134.3]))
        #print("set tool: ", self.set_tool("RG6"))
        #print("tool: ", self.get_tool())
        # tcp = self.get_tcp()
        # print("tcp: ", tcp)
        # if tcp != "TCP":
        #     print("add tcp: ", self.add_tcp(name="TCP", position=[0, 0, 0., 0, 0, 0]))
        #print("set tcp: ", self.set_tcp("RG6 TCP"))
        #print("tcp: ", self.get_tcp())
        assert self.change_collision_sensitivity(10)

        assert self.connect_rt_control(create_string_buffer(ip.encode("utf-8")), 12347), \
            f"fail to connect doosan robot with udp!"
        assert self.set_rt_control_output(create_string_buffer("v1.0".encode("utf-8")), dt, 0)
        assert self.set_velj_rt((c_float * NUM_JOINT)(*((max_vel * 1000,) * NUM_JOINT)))
        assert self.set_accj_rt((c_float * NUM_JOINT)(*((max_acc * 180 / 3.14,) * NUM_JOINT)))
        assert self.start_rt_control()

    def switch_mode(self, control_mode):
        # print(f"old control_mode: {self.control_mode}, new control_mode: {control_mode}")
        if control_mode == "normal":
            self.stop(STOP_TYPE.STOP_TYPE_SLOW)
            time.sleep(0.3)
            self.set_robot_mode(ROBOT_MODE.ROBOT_MODE_MANUAL)
            self.set_safety_mode(SAFETY_MODE.SAFETY_MODE_MANUAL, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_STOP)
        elif control_mode == "rt":
            self.set_robot_mode(ROBOT_MODE.ROBOT_MODE_AUTONOMOUS)
            self.set_safety_mode(SAFETY_MODE.SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_MOVE)
            print("rt")
        else:
            assert False

    def get_tool(self):
        get_tool = self.dll.get_tool
        get_tool.argtypes, get_tool.restype = [], c_char_p
        return string_at(get_tool()).decode("utf-8")

    def add_tool(self, name: str, weight: float, cog: List[float], inertia: List[float] = None):
        add_tool = self.dll.add_tool
        add_tool.argtypes, add_tool.restype = [c_char_p, c_float, c_float * 3, c_float * NUM_TASK], bool
        cog = np.asarray(cog)
        cog_arr = (c_float * 3)(*cog.tolist())
        if inertia is None:
            inertia = [0., ] * NUM_TASK
        inertia = np.asarray(inertia)
        inertia_arr = (c_float * NUM_TASK)(*inertia.tolist())
        return add_tool(create_string_buffer(name.encode("utf-8")), weight, cog_arr, inertia_arr)

    def set_tool(self, name):
        set_tool = self.dll.set_tool
        set_tool.argtypes, set_tool.restype = [c_char_p], bool
        return set_tool(create_string_buffer(name.encode("utf-8")))

    def get_tcp(self):
        get_tcp = self.dll.get_tcp
        get_tcp.argtypes, get_tcp.restype = [], c_char_p
        return string_at(get_tcp()).decode("utf-8")

    def add_tcp(self, name: str, position: List[float]):
        add_tcp = self.dll.add_tcp
        add_tcp.argtypes, add_tcp.restype = [c_char_p, c_float * NUM_TASK], bool
        position = np.asarray(position)
        position = (c_float * NUM_TASK)(*position.tolist())
        return add_tcp(create_string_buffer(name.encode("utf-8")), position)

    def set_tcp(self, name):
        set_tcp = self.dll.set_tcp
        set_tcp.argtypes, set_tcp.restype = [c_char_p], bool
        return set_tcp(create_string_buffer(name.encode("utf-8")))

    def get_current_posj(self):
        if not hasattr(self, "_get_current_posj"):
            self._get_current_posj = self.dll.get_current_posj
            self._get_current_posj.argtypes = []
            self._get_current_posj.restype = POINTER(ROBOT_POSE)

        p = self._get_current_posj()
        joint_pos = np.array(p.contents._fPosition)
        return joint_pos

    def get_current_velj(self):
        if not hasattr(self, "_get_current_velj"):
            self._get_current_velj = self.dll.get_current_velj
            self._get_current_velj.argtypes = []
            self._get_current_velj.restype = POINTER(ROBOT_VEL)

        p = self._get_current_velj()
        joint_vel = np.array(p.contents._fVelocity)
        return joint_vel

    def get_current_posx(self):
        if not hasattr(self, "_get_current_posx"):
            self._get_current_posx = self.dll.get_current_posx
            self._get_current_posx.argtypes = [COORDINATE_SYSTEM]
            self._get_current_posx.restype = POINTER(ROBOT_TASK_POSE)

        p = self._get_current_posx(COORDINATE_SYSTEM.COORDINATE_SYSTEM_BASE)
        actual_tcp_position = np.array(p.contents._fTargetPos)
        return actual_tcp_position

    def get_current_velx(self):
        if not hasattr(self, "_get_current_velx"):
            self._get_current_velx = self.dll.get_current_velx
            self._get_current_velx.argtypes = []
            self._get_current_velx.restype = POINTER(ROBOT_VEL)

        p = self._get_current_velx()
        actual_tcp_velocity = np.array(p.contents._fVelocity)
        return actual_tcp_velocity

    def get_joint_torque(self):
        if not hasattr(self, "_get_joint_torque"):
            self._get_joint_torque = self.dll.get_joint_torque
            self._get_joint_torque.argtypes, self._get_joint_torque.restype = [], POINTER(ROBOT_FORCE)

        p = self._get_joint_torque()
        return np.array(p.contents._fForce)

    def get_external_torque(self):
        if not hasattr(self, "_get_external_torque"):
            self._get_external_torque = self.dll.get_external_torque
            self._get_external_torque.argtypes, self._get_external_torque.restype = [], POINTER(ROBOT_FORCE)

        p = self._get_external_torque()
        return np.array(p.contents._fForce)

    def get_tool_force(self):
        if not hasattr(self, "_get_tool_force"):
            self._get_tool_force = self.dll.get_tool_force
            self._get_tool_force.argtypes, self._get_tool_force.restype = [], POINTER(ROBOT_FORCE)

        p = self._get_tool_force()
        return np.array(p.contents._fForce)

    def read_data_rt(self):
        if not hasattr(self, "_read_data_rt"):
            self._read_data_rt = self.dll.read_data_rt
            self._read_data_rt.argtypes, self._read_data_rt.restype = [], POINTER(RT_OUTPUT_DATA_LIST)
        p_rt_data_list = self._read_data_rt()
        rt_data_list = p_rt_data_list.contents
        return rt_data_list

    def movej(self, jpos, max_vel=30, max_acc=20, ):
        if not hasattr(self, "_movej"):
            self._movej = self.dll.movej
            self._movej.argtypes, self._movej.restype = [c_float * NUM_JOINT, c_float, c_float,
                                                         c_float, MOVE_MODE, c_float, BLENDING_SPEED_TYPE], c_bool
        jpos = np.asarray(jpos)
        arr_jpos = (c_float * NUM_JOINT)(*jpos.tolist())
        ret = self._movej(arr_jpos, max_vel, max_acc, 0.0, MOVE_MODE.MOVE_MODE_ABSOLUTE, 0,
                          BLENDING_SPEED_TYPE.BLENDING_SPEED_TYPE_DUPLICATE)
        return ret

    def task_compliance_ctrl(self, stiffness, target_time=0.0):
        if not hasattr(self, "_task_compliance_ctrl"):
            self._task_compliance_ctrl = self.dll.task_compliance_ctrl
            self._task_compliance_ctrl.argtypes, self._task_compliance_ctrl.restype = \
                [c_float * NUM_TASK, COORDINATE_SYSTEM, c_float], c_bool
        stiffness = np.asarray(stiffness)
        arr_stiffness = (c_float * NUM_TASK)(*stiffness.tolist())
        self._task_compliance_ctrl(arr_stiffness, COORDINATE_SYSTEM.COORDINATE_SYSTEM_BASE, target_time)

    def amovej(self, jpos, max_vel=30, max_acc=20, target_time=0.0):
        if not hasattr(self, "_amovej"):
            self._amovej = self.dll.amovej
            self._amovej.argtypes, self._amovej.restype = [c_float * NUM_JOINT, c_float, c_float,
                                                           c_float, MOVE_MODE, BLENDING_SPEED_TYPE], c_bool
        jpos = np.asarray(jpos)
        arr_jpos = (c_float * NUM_JOINT)(*jpos.tolist())
        ret = self._amovej(arr_jpos, max_vel, max_acc, target_time, MOVE_MODE.MOVE_MODE_ABSOLUTE,
                           BLENDING_SPEED_TYPE.BLENDING_SPEED_TYPE_DUPLICATE)
        return ret

    def servol(self, target_pos , target_vel, target_acc, target_time):
        if not hasattr(self, "_servol"):
            self._servol = self.dll.servol
            self._servol.argtypes, self._servol.restype = [c_float * 6, c_float * 2, c_float * 2,
                                                         c_float], c_bool
        #target_pos = np.array(target_pos)
        #arr_pos = (c_float * NUM_TASK)(*target_pos.tolist())
        print("type(arr_pos)=",type(target_pos))
        return self._servol(target_pos, target_vel, target_acc, target_time)

    def servol_rt(self, target_pos , target_vel, target_acc, target_time):
        if not hasattr(self, "_servol_rt"):
            self._servol_rt = self.dll.servol_rt
            self._servol_rt.argtypes, self._servol_rt.restype = [c_float * 6, c_float * 6, c_float * 6,
                                                         c_float], c_bool
        #target_pos = np.array(target_pos)
        #arr_pos = (c_float * NUM_TASK)(*target_pos.tolist())
        print("type(arr_pos)=",type(target_pos))
        return self._servol_rt(target_pos, target_vel, target_acc, target_time)

    def speedl_rt(self, target_vel: np.ndarray, target_acc, target_time):
        assert target_time > 0
        if not hasattr(self, "_speedl_rt"):
            self._speedl_rt = self.dll.speedl_rt
            self._speedl_rt.argtypes, self._speedl_rt.restype = [c_float * NUM_TASK, c_float * NUM_TASK, c_float], c_bool
        target_vel = np.asarray(target_vel)
        # target_vel = np.concatenate([target_vel[:3] * 1000, target_vel[3:] * 180 / 3.14])
        arr_vel = (c_float * NUM_TASK)(*target_vel.tolist())

        target_acc = np.asarray(target_acc)
        # target_acc = np.concatenate([target_acc[:3] * 1000, target_acc[3:] * 180 / 3.14])
        arr_acc = (c_float * NUM_TASK)(*target_acc.tolist())
        # print(f"speedl_rt-target_vel: {target_vel}, target_acc: {target_acc}")
        return self._speedl_rt(arr_vel, arr_acc, target_time)

    def torque_rt(self, torques: np.ndarray, target_time=0.0):
        if not hasattr(self, "_torque_rt"):
            self._torque_rt = self.dll.torque_rt
            self._torque_rt.argtypes, self._torque_rt.restype = [c_float * NUM_JOINT, c_float], c_bool
        torques = np.asarray(torques)
        arr_torque = (c_float * NUM_JOINT)(*torques.tolist())
        return self._torque_rt(arr_torque, target_time)

def axisangle2quat(vec, eps=1e-6):
    """
    Converts scaled axis-angle to quat.
    Args:
        vec (np.ndarray): (..., 3) tensor where final dim is (ax,ay,az) axis-angle exponential coordinates
        eps (float): Stability value below which small values will be mapped to 0

    Returns:
        tensor: (..., 4) tensor where final dim is (x,y,z,w) vec4 float quaternion
    """
    # store input shape and reshape
    input_shape = vec.shape[:-1]
    vec = vec.reshape((-1, 3))

    # Grab angle
    angle = np.linalg.norm(vec, axis=-1, keepdims=True)

    # Create return array
    quat = np.zeros((int(np.prod(input_shape)), 4))
    quat[:, 3] = 1.0

    # Grab indexes where angle is not zero an convert the input to its quaternion form
    idx = angle.reshape(-1) > eps  # torch.nonzero(angle).reshape(-1)
    quat[idx, :] = np.concatenate([vec[idx, :] * np.sin(angle[idx, :] / 2.0) / angle[idx, :],
                                   np.cos(angle[idx, :] / 2.0)], axis=-1)

    # Reshape and return output
    quat = quat.reshape(list(input_shape) + [4, ])
    return quat
def quat_mul(a, b):
    x1, y1, z1, w1 = a[0], a[1], a[2], a[3]
    x2, y2, z2, w2 = b[0], b[1], b[2], b[3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return np.array([x, y, z, w])
def quat_inv(q):
    conjugate = quat_conjugate(q)
    norm_q = np.linalg.norm(q)
    inv_q = conjugate / (norm_q + 1e-6)
    return inv_q
def quat_conjugate(q):
    conjugate = np.zeros_like(q)
    conjugate[:3] = -q[:3]
    conjugate[3] = q[3]
    return conjugate

def dh_transform(theta, d, a, alpha):
    """ 计算 DH 变换矩阵 """
    T = np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
        [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
        [0, 0, 0, 1]
    ])
    return T

def transform_force_torque(force_torque, transform):
    """ 将六维力数据转换到基坐标系 """
    force = force_torque[:3]
    rotation = transform[:3, :3]
    force_base = rotation @ force
    return np.array(force_base)

def forward_kinematics(dh_params, joint_angles):
    """ 计算末端执行器相对于基坐标系的变换矩阵 """
    T_0_6 = np.eye(4)
    # 逐个关节计算变换矩阵并乘以总变换矩阵
    for i in range(6):
        theta, d, a, alpha, offset = dh_params[i]
        # 对应的theta是关节的角度与offset相加
        T_i = dh_transform(joint_angles[i] + offset + theta, d, a, alpha)
        T_0_6 = np.dot(T_0_6, T_i)
    # 输出末端到基座的变换矩阵
    # np.set_printoptions(precision=4, suppress=True)
    print(T_0_6)
    # 点 p 在末端坐标系下的表示 (3x1)，这里假设 p 在末端坐标系下为 [x, y, z]
    p = np.array([0, 0, 0.02])  # 假设传感器在末端坐标系下的相对位置为[0, 0, 10]
    # 构建平移矩阵 T_translation (4x4)，将末端坐标系原点平移到 p
    T_translation = np.eye(4)
    T_translation[0, 3] = p[0]
    T_translation[1, 3] = p[1]
    T_translation[2, 3] = p[2]
    # 最终的变换矩阵：从以 p 为原点的坐标系到基座坐标系
    T_sensor_to_base = np.dot(T_0_6, T_translation)
    return T_sensor_to_base



if __name__ == "__main__":
    # ----------连接传感器-----------
    sensor_ip = "192.168.1.102"  # 六维力传感器 IP 地址
    driver = ForceTorqueSensorDriver(sensor_ip)
    driver.start()
    print("Calibration data:")
    print(driver.get_calibration_data())  # 传感器校准
    print("Sensor configuration:")
    print(driver.get_sensor_configuration())  # 传感器配置

    # 机械臂 DH 参数定义
    dh_params = [
        (0, 0.1525, 0, 0, 0),
        (0, 0.0345, 0, -np.pi / 2, -np.pi / 2),
        (0, 0, 0.411, 0, np.pi / 2),
        (0, 0.368, 0, np.pi / 2, 0),
        (0, 0, 0, -np.pi / 2, 0),
        (0, 0.121, 0, np.pi / 2, 0)
    ]

    # 数据文件初始化
    dt = datetime.now()
    nowtime_str = dt.strftime('%y-%m-%d %I-%M-%S')  # 时间
    filename = nowtime_str + '_combined_data.csv'
    f = open(filename, 'a+', newline='')
    writer = csv.writer(f)
    writer.writerow(['ch0', 'ch1', 'ch2', 'ch3', 'ch4', 'ch5', 'ch6', 'ch7', 'tcp_x', 'tcp_y', 'tcp_z', 'force_x', 'force_y', 'force_z'])

    # 初始化和连接机械臂
    ip = "192.168.1.105"
    api = DoosanAPI(ip, dt=0.002)
    api.stop(STOP_TYPE.STOP_TYPE_SLOW)
    time.sleep(0.3)
    api.set_robot_mode(ROBOT_MODE.ROBOT_MODE_MANUAL)
    assert api.set_safety_mode(SAFETY_MODE.SAFETY_MODE_MANUAL, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_STOP)

    api.movej(np.array([-106, -58, -48, 1, -70, -90]))
    print("set joint")
    time.sleep(10)

    api.set_robot_mode(ROBOT_MODE.ROBOT_MODE_AUTONOMOUS)
    assert api.set_safety_mode(SAFETY_MODE.SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_MOVE)

    # 传感器归零
    sensor_1 = driver.get_wrench()
    if sensor_1:
        x_1 = sensor_1[0]
        y_1 = sensor_1[1]
        z_1 = sensor_1[2]

    # 初始化肌电信号采集
    myo.init()
    hub = myo.Hub()
    listener = EmgCollector(512)

    with hub.run_in_background(listener.on_event):
        while True:
            start_time = time.perf_counter()

            # 获取机械臂实时数据
            rt_output_data_list = api.read_data_rt()
            joint_angles_1 = np.array(rt_output_data_list.actual_joint_position, dtype=float)
            joint_angles = joint_angles_1 * np.pi / 180
            actual_p = np.array(rt_output_data_list.actual_tcp_position, dtype=float)  # 欧拉角ZYZ表示

            # 传感器数据转换到基座标系
            sensor_FT = driver.get_wrench()
            sensor_FT[0] = sensor_FT[0] - x_1
            sensor_FT[1] = sensor_FT[1] - y_1
            sensor_FT[2] = sensor_FT[2] - z_1
            transform = forward_kinematics(dh_params, joint_angles)
            base_TF = transform_force_torque(sensor_FT, transform)

            # 数据记录
            emg_data = listener.get_emg_data()
            if emg_data:
                latest_emg = emg_data[-1][1]  # 获取最新的肌电数据
            else:
                latest_emg = [0] * 8  # 如果没有数据，填充 0

            row = list(latest_emg) + list(actual_p[:3]) + list(base_TF[:3])
            writer.writerow(row)

            # 控制机械臂
            delta_X = 0.0
            delta_Y = 0.0
            delta_Z = 0.0
            ee = actual_p
            ee[0] += delta_X
            ee[1] += delta_Y
            ee[2] += delta_Z

            target_vel = np.array([5.0, 5.0, 5, 1, 1, 1], dtype=float)
            target_acc = np.array([1, 1, 1, 1, 1, 1], dtype=float)
            ee = np.array(ee).tolist()
            float_array_type1 = ctypes.c_float * 6
            pos = float_array_type1(*ee)
            float_array_type2 = ctypes.c_float * 6
            target_vel = float_array_type2(*target_vel)
            float_array_type3 = ctypes.c_float * 6
            target_acc = float_array_type3(*target_acc)
            api.servol_rt(pos, target_vel, target_acc, 0.2)

            # 控制循环时间
            while (time.perf_counter() - start_time) < 0.005:
                pass
