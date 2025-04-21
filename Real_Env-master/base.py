import time
#import cv2
#import gym
import enum
import threading
import queue
import numpy as np
from controller import *


class BaseGripperComponent:

    def __init__(self):
        self.pause_event = None
        self.sync_thread = None
        self.action = None
        self.lock = threading.Lock()

    def initialize(self) -> bool:
        self.pause_event = threading.Event()
        self.pause_event.set()
        self.sync_thread = threading.Thread(target=self.gripper_sync_task)
        self.sync_thread.daemon = True
        self.sync_thread.start()
        return True

    def reset(self):
        raise NotImplemented

    def control(self, action):
        self.action = action
        # self._sync(action=self.action)
        return True

    def _sync(self, action) -> bool:
        raise NotImplemented

    def gripper_sync_task(self):
        while True:
            # print("gripper sync")
            self.pause_event.wait()
            if self.action is not None:
                self._sync(action=self.action)
            time.sleep(0.1)

    @property
    def action_space(self):
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(1,))


class Command(enum.Enum):
    STOP = 0
    MOVEJ = 1
    SET_GOAL = 2


class RobotControlMode(enum.Enum):
    NORMAL = 0
    RT = 1
    TEACH = 2


class BaseRobotComponent(threading.Thread):

    def __init__(
            self,
            urdf="",
            controller_type="eac_pos",
            frequency=500,
            input_queue_size=12,
            state_queue_size=12,
    ):
        super().__init__(name="RTRobotComponent")
        controller_config = {
            "n_dof": self.n_dof,
            "rest_qpos": self.rest_qpos,
        }
        if controller_type is not None:
            self.controller = load_controller(controller_type=controller_type, controller_config=controller_config)
            self.command_dim = self.controller.command_dim
        else:
            self.controller = None

        self.urdf = urdf

        # if self.urdf != "":
        #     MR = loadURDF(URDF_PATH[self.urdf])
        #     self.Slist = MR["Slist"]
        #     self.Mlist = MR["Mlist"]
        #     self.Glist = MR["Glist"]
        #     self.Blist = MR["Blist"]

        self.frequency = frequency
        self.dt = 1.0 / frequency
        self.control_mode: RobotControlMode = None

        self.lock = threading.RLock()
        self.busy_event = threading.Event()
        self.input_queue = queue.Queue(maxsize=input_queue_size)
        self.state_queue = queue.Queue(maxsize=state_queue_size)
        self.control_function_mapping = {}

    def start(self, wait=True):
        super().start()
        if wait:
            self.start_wait()
        print(f"[RTRobotComponent] Controller thread spawned")

    def stop(self, wait=True):
        message = {
            "cmd": Command.STOP.value,
            "action": None,
            "target_time": time.time(),
            "timestamp": time.time(),
        }
        self.input_queue.put(message)
        if wait:
            self.stop_wait()

    def start_wait(self):
        assert self.is_alive()

    def stop_wait(self):
        self.join()

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def switch_mode(self, control_mode: RobotControlMode):  # ["normal", "rt", "teach"]
        if self.control_mode != control_mode:
            self._switch_mode(control_mode=control_mode)
            self.control_mode = control_mode

    def _switch_mode(self, control_mode: RobotControlMode):  # ["normal", "rt", "teach"]
        raise NotImplemented

    def register_control_function(self, control_type, func):
        self.control_function_mapping[control_type] = func

    def control(self, cmd: Command, action=None, duration=0.0):
        #print("7777777")
        cur_time = time.time()
        message = {
            "cmd": cmd,
            "action": action,
            "duration": duration,
            "target_time": cur_time + duration,
            "timestamp": cur_time,
        }

        if self.input_queue.full():
            self.input_queue.get()
            # print(f"input_queue full!")
        self.input_queue.put(message)

    def get_control_dict(self):
        raise NotImplemented

    def run(self):
        print("888888888")
        keep_running = True
        last_time = None
        target_time = None
        self.control_mode = None

        while keep_running:
            print("99999999999")
            control_dict = self.get_control_dict()

            if self.state_queue.full():
                self.state_queue.get()
            self.state_queue.put(control_dict)

            cur_time = time.time()
            if last_time is None:
                last_time = cur_time
            else:
                freq = 1 / (cur_time - last_time + 1e-6)
                last_time = cur_time
                # print(f"[RTRobotComponent] Actual frequency {round(freq, 3)}HZ")

            if self.control_mode == RobotControlMode.RT.value and self.controller is not None:
                print("10101010")
                output = self.controller.compute_control(control_dict=control_dict, cur_time=cur_time)
                if output is not None:
                    # print(f"control_type: {self.controller.control_type}, {output}")
                    self.control_function_mapping[self.controller.control_type](output)
            elif self.control_mode == RobotControlMode.TEACH.value:
                gravity_torque = control_dict["gravity_torque"]
                #print("gravity_torque=",gravity_torque)
                self.control_function_mapping["joint_torque_rt"](gravity_torque)
                target_time = None

            if target_time is not None and cur_time < target_time:
                self.busy_event.set()
                continue

            if self.input_queue.empty():
                target_time = cur_time + self.dt
                self.busy_event.clear()
                continue

            command = self.input_queue.get()

            cmd = command["cmd"]
            if cmd == Command.MOVEJ.value:
                self.busy_event.set()
                action = command["action"]
                jpos = action if action is not None else self.rest_qpos
                self.switch_mode(control_mode=RobotControlMode.NORMAL.value)
                self.control_function_mapping["joint_pos"](jpos)
                control_dict = self.get_control_dict()
                if self.controller is not None:
                    self.controller.reset(control_dict=control_dict)
                self.busy_event.clear()
            elif cmd == Command.SET_GOAL.value:
                print("10")
                self.switch_mode(control_mode=RobotControlMode.RT.value)
                action = command["action"]
                target_time = cur_time + command["duration"] + self.dt

                if target_time <= cur_time:
                    print(f"target_time: [{round(target_time, 3)}] < cur_time: [{round(cur_time, 3)}], skip!")
                    continue
                print("1212121212")
                self.controller.update_goal(
                    control_dict=control_dict,
                    command=action,
                    target_time=target_time,
                    cur_time=cur_time,
                )
            elif cmd == Command.STOP.value:
                keep_running = False
                # stop immediately, ignore later commands
            else:
                assert False

    @property
    def rest_qpos(self):
        raise NotImplemented

    @property
    def position_limits(self):  # [2, n=3]
        raise NotImplemented

    @property
    def n_dof(self):
        raise NotImplemented

'''
class RealEnv(gym.Env):

    def __init__(
            self,
            robot_component: BaseRobotComponent,
            gripper_component: BaseGripperComponent,
            camera_kwargs: dict,  # camera_id, lt, rb, width, height
            control_freq=10,
    ):
        super().__init__()
        self.robot_component = robot_component
        self.gripper_component = gripper_component
        self.camera_kwargs = camera_kwargs
        self.control_freq = control_freq  # hz
        self.control_period = 1. / self.control_freq  #

        self.camera_names = ["real"]
        self.real_camera = None
        self.now_image = None

        assert self.initialize_camera(), f"fail to initialize camera with id {self.camera_kwargs['camera_id']}!"
        print("success to initialize camera")
        self.robot_component.daemon = True
        self.robot_component.start()
        print("success to initialize robot")
        if self.gripper_component is not None:
            assert self.gripper_component.initialize(), "fail to initialize gripper!"
            print("success to initialize gripper")

    def reset(self):
        self.robot_component.control(cmd=Command.MOVEJ.value)
        if self.gripper_component is not None:
            ret2 = self.gripper_component.reset()
        input("Please make sure reset.......")
        self.robot_component.busy_event.wait()
        return self._get_observation()

    def step(self, action):
        robot_action = action[:self.robot_component.command_dim]
        gripper_action = action[self.robot_component.command_dim:]
        ret1 = True
        if self.gripper_component is not None:
            ret1 = self.gripper_component.control(gripper_action)
        self.robot_component.control(cmd=Command.SET_GOAL.value, action=robot_action, duration=self.control_period)
        target_time = time.time() + self.control_period
        self.robot_component.busy_event.wait()
        while time.time() < target_time:
            time.sleep(self.robot_component.dt)
        return self._get_observation(), None, self._check_terminated(), {}

    def initialize_camera(self) -> bool:
        self.real_camera = cv2.VideoCapture(self.camera_kwargs["camera_id"])
        return self.real_camera.isOpened()

    def _get_observation(self):
        robot_state_dict = self.robot_component.state_queue.get()
        robot_state_dict = {key: v for key, v in robot_state_dict.items() if isinstance(v, np.ndarray)}
        for _ in range(10):
            self.real_camera.grab()
        ret, frame = self.real_camera.read()
        assert self.real_camera.isOpened() and ret and frame is not None
        h, w, c = frame.shape
        lt, rb = self.camera_kwargs["lt"], self.camera_kwargs["rb"]
        assert (rb[0] - lt[0]) < w and (rb[1] - lt[1]) < h, f"{(h, w)}"
        # print(h, w, c)
        self.now_image = frame[:, :, ::-1]
        clipped_frame = self.now_image[lt[1]: rb[1], lt[0]: rb[0], :]
        resized_frame = cv2.resize(clipped_frame, (self.camera_kwargs["width"], self.camera_kwargs["height"]))
        return {"observation": {"real_image": resized_frame, **robot_state_dict}}

    def _check_terminated(self):
        return False

    def render(self, *args, **kwargs):
        frame = self.now_image.copy()
        lt, rb = self.camera_kwargs["lt"], self.camera_kwargs["rb"]
        cv2.rectangle(frame, lt, rb, (255, 0, 0), 2)
        return {"real_image": frame}

    @property
    def action_space(self):
        robot_low = self.robot_component.controller.input_min
        robot_high = self.robot_component.controller.input_max
        gripper_low = [-1.0]
        gripper_high = [1.0]
        low = np.concatenate([robot_low, gripper_low])
        high = np.concatenate([robot_high, gripper_high])
        return gym.spaces.Box(low=low, high=high)'''
