import math
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

#对机械臂进行示教并保存关节的位置信息
def teach(robot_ip, freq=50):
    dt = 1.0 / freq
    #teach_dataset = DynamicsDataset(name="doosan_teach")


    doosan_robot = DoosanComponent(
        controller_type=None,
        robot_ip=robot_ip,
    )
    doosan_robot.start()
    time.sleep(1)
    rest_qpos = doosan_robot.rest_qpos
    print("start to reset...")
    doosan_robot.control(cmd=Command.MOVEJ.value, action=rest_qpos)
    time.sleep(3)
    doosan_robot.busy_event.wait()
    print("reset success")
    doosan_robot.switch_mode(RobotControlMode.TEACH.value)
    #time.sleep(20)
    #print(f"joint_pos: {control_dict['joint_pos']}")

    for i in range(0, 100000):
        control_dict = doosan_robot.state_queue.get()
        print(f"joint_pos: {control_dict['joint_pos']}")
        #teach_dataset.put(control_dict)
        time.sleep(dt)
        if i % 20 == 0 and i != 0:
            #teach_dataset.end()
            print(i)
    #teach_dataset.save()
def compute_6_by_6_diagonal_matrix_two_value(val1, val2):
    if np.isscalar(val1):
        assert np.isscalar(val2)
        aux_m = np.diag([val1, val1, val1, val2, val2, val2])
    else:
        val1 = np.asarray(val1)
        val2 = np.asarray(val2)

        aux_m = np.zeros((6, 6))
        aux_m[:3, :3] = val1
        aux_m[3:, 3:] = val2
    return aux_m


def critical_damping_formula(m, k):
    """Compute the critical damping.

        Parameters:
        m (int/float/array/np.array): The mass.
        k (int/float/array/np.array): The k parameter.

        Returns:
        np.ndarray/float: The computed damping

       """
    assert type(m) == type(k)
    if np.isscalar(m):
        aux_d = 2 * math.sqrt(m*(k+1))
    else:
        org_length = len(m)
        m = np.asarray(m)
        k = np.asarray(k)

        aux_d = 2 * np.sqrt(m * (k + np.eye(org_length)))

    return aux_d


def replay(robot_ip, freq=200):
    dt = 1.0 / freq
    #teach_dataset = DynamicsDataset(name="doosan_teach")

    doosan_robot = DoosanComponent(
        controller_type="eac_pos",
        robot_ip=robot_ip,

    )
    #doosan_robot.start()
    time.sleep(1)
    print("start to position...")
    rest_qpos = doosan_robot.rest_qpos
    doosan_robot.movej(rest_qpos)#将机械臂移动到初始采集点的位置
    time.sleep(3)
    print("ready...")
    t1=time.time()
    doosan_robot.switch_mode(control_mode=RobotControlMode.RT.value)
    mp = 0.5
    kp = 150
    mo = 1
    ko = 15
    kp2 = 80
    ko2 = 80
    mp2 = 80
    mo2 = 80
    max_pos_acceleration = 0.3
    actual_speed = np.zeros(6)  # [6]
    actual_acceleration = np.zeros(6)
    mass_matrix = compute_6_by_6_diagonal_matrix_two_value(mp, mo)
    k_matrix = compute_6_by_6_diagonal_matrix_two_value(kp, ko)
    aux_dp = critical_damping_formula(mp2, kp2)
    aux_do = critical_damping_formula(mo2, ko2)
    damp_matrix = compute_6_by_6_diagonal_matrix_two_value(aux_dp, aux_do)

    mass_matrix = np.asarray(mass_matrix)  # [3, 3]
    inv_mass = np.linalg.inv(mass_matrix)  # [3, 3]
    k_matrix = np.asarray(k_matrix)  # [3, 3]
    damp_matrix = np.asarray(damp_matrix)  # [3, 3]
    inv_damp_matrix = np.linalg.inv(damp_matrix)
    inv_k_matrix = np.linalg.inv(k_matrix)
    max_pos_acceleration = max_pos_acceleration
    #goal_pos=rest_qpos
    control_dict = doosan_robot.get_control_dict()
    eef_p = control_dict["eef_pos_quat"]
    eef_ex_force = control_dict["eef_ex_force"]
    ee_ = eef_p[0: 2]
    goal_pos = ee_
    print("goal_pos=",goal_pos)
    print("eef_ex_force=", eef_ex_force)
    time.sleep(10)
    print("set....goal")
    K1=150
    K2=150
    B1=50
    B2=50

    while time.time()-t1<=30:
        control_dict = doosan_robot.get_control_dict()
        eef_pos_quat = control_dict["eef_pos_quat"]
        eef_ex_force = control_dict["eef_ex_force"]
        print("eef_ex_force=", eef_ex_force)
        ex_torque = np.linalg.norm(eef_ex_force[0: 2])
        print("ex_torque", ex_torque)
        if ex_torque > 40 or ex_torque < 5:
            _wrench_external = np.zeros_like(eef_ex_force)
        else:
            _wrench_external = np.asarray(eef_ex_force)

        delx=eef_pos_quat[0]-eef_p[0]
        v1= (_wrench_external[0]-K1*delx)/B1
        dely = eef_pos_quat[1] - eef_p[1]
        v2 = (_wrench_external[1] - K2 * dely) / B2
        actual_speed=np.array([v1,v2,0,0,0,0])
        print(actual_speed)
        doosan_robot.eef_speed_rt(actual_speed)
        #time.sleep(1/500)
'''


        ee_pos = eef_pos_quat[0: 2]
        pos_error1 = ee_pos - goal_pos  # [3]
        print("pos_error1=",pos_error1)
        pos_error2 = np.concatenate([pos_error1, [0.0,0.0, 0.0, 0.0]])
        delta_pose = pos_error2[:6]
        print("delta_pose=",delta_pose)

        coupling_wrench_arm = np.dot(k_matrix, delta_pose)  # [3] or [6]
        print("coupling_wrench_arm=",coupling_wrench_arm)
        BV=_wrench_external - coupling_wrench_arm
        print("BV=",BV)
        actual_speed = np.dot(inv_damp_matrix,  BV)
        print("actual_speed=", actual_speed)
        actual_speed2 = actual_speed[0: 2]
        actual_speed3 = np.concatenate([actual_speed2, [0.0, 0.0, 0.0, 0.0]])
        print("actual_speed3=", actual_speed3)
        #v=postprocess_control(actual_speed)
        doosan_robot.eef_speed_rt(actual_speed3)

'''


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    robot_ip = "192.168.5.110"
    #teach(robot_ip=robot_ip, freq=20)
    replay(robot_ip=robot_ip, freq=200)
