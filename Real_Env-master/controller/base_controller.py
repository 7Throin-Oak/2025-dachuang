# ---------------------------------------------------------------
# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# This work is licensed under the NVIDIA Source Code License
# for OSCAR. To view a copy of this license, see the LICENSE file.
# ---------------------------------------------------------------
from typing import Union
from collections import Iterable
import numpy as np


class Controller:
    """
    Base controller from which all controllers extend from. This class includes basic APIs child controllers
    should adhere to.

    In general, the controller pipeline is as follows:

    received cmd --> clipped --> scaled --> processed through controller --> clipped --> normalized (optional)

    Args:
        command_dim (int): input dimension (i.e.: dimension of received commands)
        input_min (int, float, or array): Minimum values below which received commands will be clipped
        input_max (int, float, or array): Maximum values above which received commands will be clipped
        output_min (int, float, or array): Lower end of range that received commands will be mapped to
        output_max (int, float, or array): Upper end of range that received commands will be mapped to
        control_min (int, float, or array): Minimum control values below which outputted controls will be clipped
        control_max (int, float, or array): Maximum control values above which outputted controls will be clipped
        control_dim (int): Outputted control dimension
    """

    def __init__(
            self,
            command_dim: int,
            input_min: Union[float, Iterable],
            input_max: Union[float, Iterable],
            output_min: Union[float, Iterable],
            output_max: Union[float, Iterable],
            control_min: Union[float, Iterable],
            control_max: Union[float, Iterable],
            control_dim: int,
    ):
        # Store dimensions
        self.command_dim = command_dim
        self.control_dim = control_dim

        # Store limits
        self.input_min = self.nums2array(nums=input_min, dim=self.command_dim)
        self.input_max = self.nums2array(nums=input_max, dim=self.command_dim)
        self.output_min = self.nums2array(nums=output_min, dim=self.command_dim)
        self.output_max = self.nums2array(nums=output_max, dim=self.command_dim)
        self.control_min = self.nums2array(nums=control_min, dim=self.control_dim)
        self.control_max = self.nums2array(nums=control_max, dim=self.control_dim)

        # Initialize other internal variables
        self.command_scale = None
        self.command_output_transform = None
        self.command_input_transform = None
        self.control_normalization_scale = None
        self.control_input_transform = None

    def scale_command(self, command):
        """
        Clips @command to be within self.input_min and self.input_max, and then re-scale the values to be within
        the range self.output_min and self.output_max
        Args:
            command (np.ndarray): Command to scale
        Returns:
            np.ndarray: Re-scaled command
        """
        # Only calculate command scale once if we havne't done so already
        if self.command_scale is None:
            self.command_scale = abs(self.output_max - self.output_min) / abs(self.input_max - self.input_min)
            self.command_output_transform = (self.output_max + self.output_min) / 2.0
            self.command_input_transform = (self.input_max + self.input_min) / 2.0
        command = np.clip(command, self.input_min, self.input_max)
        transformed_command = (command - self.command_input_transform) * self.command_scale + \
                              self.command_output_transform

        return transformed_command

    def postprocess_control(self, control):
        """
        Clips @control to be within range [self.control_min, self.control_max], and optionally normalizes the commands
        to be within range [-1, 1] if self.normalize_control is True. Assumes final dim of @control is the relevant
        control dimension

        Args:
            control (np.ndarray): Raw control computed from controller

        Returns:
            tensor: Clipped and potentially normalized control
        """
        # Clamp control signal
        pp_control = np.clip(control, self.control_min, self.control_max)
        return pp_control

    @staticmethod
    def nums2array(nums, dim):
        """
        Converts input @nums into torch tensor of length @dim. If @nums is a single number, broadcasts input to
        corresponding dimension size @dim before converting into torch tensor

        Args:
            nums (float or array): Numbers to map to tensor
            dim (int): Size of array to broadcast input to

        Returns:
            torch.np.ndarray: Mapped input numbers
        """
        # Make sure the inputted nums isn't a string
        assert not isinstance(nums, str), "Only numeric types are supported for this operation!"
        out = np.asarray(nums)[:dim] if isinstance(nums, Iterable) else np.ones(dim) * nums
        return out

    def reset(self, control_dict):
        """
        Reset the internal vars associated with this controller

        Args:
            control_dict (dict): Dictionary of state tensors including relevant info for controller computation

                Expected keys:
                    eef_pos_quat: shape of (7), the (lin_pos, quat_ori) state of the eef body
                    eef_vel: shape of (6), the (lin_vel, ang_vel) state of the eef body
                    joint_pos: shape of (n_dof), the (joint_pos) state of the eef body
                    joint_vel: shape of (n_dof), the (joint_vel) state of the eef body
                    joint_acc: shape of (n_dof), the (joint_acc) state of the eef body
                    joint_torque: shape of (n_dof),
                    motor_torque: shape of (n_dof),
                    mass_matrix: shape of (N_dof, N_dof), current mass matrix
                    j_eef: shape of (6, N_dof), current jacobian matrix for end effector frame
                    gravity_torque: shape of (N_dof),
                    position_limits: shape of (2, N_dof),

                Note that N_dof can be greater than control_dim; we assume the first control_dim indexes correspond to
                    the relevant elements to be used for the ik computations
        """
        raise NotImplementedError

    def update_goal(self, control_dict, command, target_time, cur_time):
        """
        Updates the internal goal based on the inputted command

        Args:
            control_dict (dict): Dictionary of keyword-mapped arrays including relevant control
                information (eef state, q states, etc.)
                Expected keys:
                    eef_pos_quat: shape of (7), the (lin_pos, quat_ori) state of the eef body
                    eef_vel: shape of (6), the (lin_vel, ang_vel) state of the eef body
                    joint_pos: shape of (n_dof), the (joint_pos) state of the eef body
                    joint_vel: shape of (n_dof), the (joint_vel) state of the eef body
                    joint_acc: shape of (n_dof), the (joint_acc) state of the eef body
                    joint_torque: shape of (n_dof),
                    motor_torque: shape of (n_dof),
                    mass_matrix: shape of (N_dof, N_dof), current mass matrix
                    j_eef: shape of (6, N_dof), current jacobian matrix for end effector frame
                    gravity_torque: shape of (N_dof),
                    position_limits: shape of (2, N_dof),
            command (ndarray): action (specific to controller)
            target_time (float):
            cur_time (float):
        """
        raise NotImplementedError

    def compute_control(self, control_dict, cur_time):
        """
        Computes low-level torque controls using internal eef goal pos / ori.

        Args:
            control_dict (dict): Dictionary of state tensors including relevant info for controller computation
            cur_time (float):
                Expected keys:
                    eef_pos_quat: shape of (7), the (lin_pos, quat_ori) state of the eef body
                    eef_vel: shape of (6), the (lin_vel, ang_vel) state of the eef body
                    joint_pos: shape of (n_dof), the (joint_pos) state of the eef body
                    joint_vel: shape of (n_dof), the (joint_vel) state of the eef body
                    joint_acc: shape of (n_dof), the (joint_acc) state of the eef body
                    joint_torque: shape of (n_dof),
                    motor_torque: shape of (n_dof),
                    mass_matrix: shape of (N_dof, N_dof), current mass matrix
                    j_eef: shape of (6, N_dof), current jacobian matrix for end effector frame
                    gravity_torque: shape of (N_dof),
                    position_limits: shape of (2, N_dof),

                Note that N_dof can be greater than control_dim; we assume the first control_dim indexes correspond to
                    the relevant elements to be used for the osc computations

        Returns:
            ndarray: Processed low-level torque control actions
        """
        raise NotImplementedError

    @property
    def goal_dim(self):
        """
        Dimension of the (flattened) goal state for this controller

        Returns:
            int: Flattened goal dimension
        """
        raise NotImplementedError

    @property
    def control_type(self):
        """
        Defines the low-level control type this controller outputs. Should be one of gymapi.DOF_MODE_XXXX

        Returns:
            int: control type outputted by this controller
        """
        raise NotImplementedError

    @property
    def required_states(self):
        raise NotImplementedError

