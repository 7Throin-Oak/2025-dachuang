import numpy as np
from .rg_client import RGClient
from envs.real_env.base import BaseGripperComponent


class RGComponent(BaseGripperComponent):

    def __init__(self, gripper_ip, speed=0.3):
        super().__init__()
        self.gripper_ip = gripper_ip
        self.speed = speed
        self.client = None
        self.cur_pos = None

    def initialize(self) -> bool:
        self.client = RGClient(gripper_type="rg6", ip=self.gripper_ip, port=502)
        success = super(RGComponent, self).initialize()
        return (self.client is not None) and success

    def reset(self):
        with self.lock:
            self.client.move_gripper(int(600))
            self.cur_pos = 0.0

    def _sync(self, action) -> bool:
        action = np.asarray(action)
        assert len(action) == 1, f"{action}"
        # print(f"rg control: {gripper_pos}")
        self.cur_pos = np.clip(
            self.cur_pos + np.array([1.0]) * self.speed * np.sign(action), -0.5, 0.5
        )
        real_v = (self.cur_pos + 0.5) * 800 + 200
        self.client.move_gripper(int(real_v))
        return True


