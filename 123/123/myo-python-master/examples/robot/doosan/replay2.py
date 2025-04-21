import os
import time
import numpy as np
from envs.real_env.robot.doosan.doosan_component import DoosanComponent
from envs.real_env.base import Command, RobotControlMode
from envs.real_env.data_utils import DynamicsDataset

GLOBAL_PATH = os.path.dirname(__file__)


def joint_pos_clip(jpos):
    return np.clip(
        jpos, np.array([-3.1, -1.6, -2.3, -3.14, -2.3, -3.1]), np.array([3.1, 1.6, 2.3, 3.1, 2.3, 3.1])
    )


def teach(robot_ip, freq=50):
    dt = 1.0 / freq
    teach_dataset = DynamicsDataset(name="doosan_teach")
    rest_qpos = teach_dataset[-1]["joint_pos"] if len(teach_dataset) > 0 else None

    doosan_robot = DoosanComponent(
        controller_type=None,
        robot_ip=robot_ip,
    )
    doosan_robot.start()
    time.sleep(1)

    print("start to reset...")
    doosan_robot.control(cmd=Command.MOVEJ.value, action=rest_qpos)
    time.sleep(3)
    doosan_robot.busy_event.wait()
    print("reset success")
    doosan_robot.switch_mode(RobotControlMode.TEACH.value)
    for i in range(0, 100000):
        control_dict = doosan_robot.state_queue.get()
        print(f"joint_pos: {control_dict['joint_pos']}")
        teach_dataset.put(control_dict)
        time.sleep(dt)
        if i % 20 == 0 and i != 0:
            teach_dataset.end()
    teach_dataset.save()


def replay(robot_ip, skip, freq=50):
    dt = 1.0 / freq
    teach_dataset = DynamicsDataset(name="doosan_teach")

    doosan_robot = DoosanComponent(
        controller_type="joint_pos",
        robot_ip=robot_ip,
    )
    doosan_robot.start()
    time.sleep(1)

    print("start to replay...")
    start = 0
    doosan_robot.control(cmd=Command.MOVEJ.value, action=joint_pos_clip(teach_dataset[start]["joint_pos"]))
    time.sleep(3)
    doosan_robot.busy_event.wait()
    # time.sleep(100)
    offset = np.zeros(6)
    for i in range(start, len(teach_dataset), skip):
        q = teach_dataset[i]["joint_pos"]
        noise = np.random.randint(low=-20, high=20, size=(6,))
        offset = np.clip(offset + noise, np.array([-20, 10, 4, 0, -10, -50]), np.array([20, 20, 30, 0, 10, 50]))
        act_q = q + offset * 3.14 / 180

        print(skip, len(teach_dataset), i, i - start, f"{act_q * 180 / 3.14}")
        doosan_robot.control(cmd=Command.SET_GOAL.value, action=joint_pos_clip(act_q), duration=10 * dt)
        doosan_robot.busy_event.wait()
        time.sleep(dt)


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    robot_ip = "192.168.5.110"
    # teach(robot_ip=robot_ip, freq=20)
    replay(robot_ip=robot_ip, skip=1, freq=10)
