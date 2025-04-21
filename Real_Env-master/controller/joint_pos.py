import os
import numpy as np
##from .interpolators import LinearInterpolator
from .base_controller import Controller
##from ..data_utils import DynamicsDataset


GLOBAL_PATH = os.path.dirname(__file__)


class JointPosController(Controller):

    def __init__(
            self,
            input_min,
            input_max,
            output_min,
            output_max,
            control_min,
            control_max,
            n_dof,
            **kwargs,
    ):
        # Run super init first
        super().__init__(
            command_dim=n_dof,
            input_min=input_min,
            input_max=input_max,
            output_min=output_min,
            output_max=output_max,
            control_min=control_min,
            control_max=control_max,
            control_dim=n_dof,
        )
        self.q_interpolator = None
        self.goal_joint_pos = None
        self.last_waypoint_time = None
        self.dataset = DynamicsDataset(name="joint_pos")
        print("-----------------joint pos---------------------")

    def reset(self, control_dict):
        self.goal_joint_pos = control_dict["joint_pos"]
        self.q_interpolator = None

    def update_goal(self, control_dict, command, target_time, cur_time):
        self.dataset.end()
        if self.q_interpolator is None:
            if self.goal_joint_pos is None:
                self.goal_joint_pos = np.asarray(command)
            self.q_interpolator = LinearInterpolator(
                times=[cur_time],
                poses=[self.goal_joint_pos],
                pose_type="joint_pos",
            )
            self.last_waypoint_time = cur_time

        self.goal_joint_pos = np.asarray(command)

        self.q_interpolator = self.q_interpolator.schedule_waypoint(
            pose=self.goal_joint_pos,
            target_time=target_time,
            cur_time=cur_time,
            last_waypoint_time=self.last_waypoint_time,
        )
        self.last_waypoint_time = cur_time

    def compute_control(self, control_dict, cur_time):
        if self.q_interpolator is None:
            return
        self.dataset.put(control_dict)
        return self.q_interpolator(cur_time)

    @property
    def required_states(self):
        return []

    @property
    def control_type(self):
        return "joint_pos_rt"
