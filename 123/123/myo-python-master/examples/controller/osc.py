from typing import Dict
import numpy as np
from .control_utils import jit_decorator, quat_mul, quat2mat, orientation_error, axisangle2quat
#from .interpolators import LinearInterpolator
from .base_controller import Controller


class OSCController(Controller):
    """
    Operational Space Controller. Leverages impedance-based end effector control.

    This controller expects 6DOF delta commands (dx, dy, dz, dax, day, daz), where the delta orientation
    commands are in axis-angle form, and outputs low-level torque commands.

    Gains may also be considered part of the action space as well. In this case, the action space would be:
        (
            dx, dy, dz, dax, day, daz                       <-- 6DOF delta eef commands
            [, kpx, kpy, kpz, kpax, kpay, kpaz]             <-- kp gains
            [, drx dry, drz, drax, dray, draz]              <-- damping ratio gains
            [, kpnx, kpny, kpnz, kpnax, kpnay, kpnaz]       <-- kp null gains
        )

    Note that in this case, we ASSUME that the inputted gains are normalized to be in the range [-1, 1], and will
    be mapped appropriately to their respective ranges, as defined by XX_limits

    Alternatively, parameters (in this case, kp or damping_ratio) can either be set during initialization or provided
    from an external source; if the latter, the control_dict should include the respective parameter(s) as
    a part of its keys

    Args:
        input_min (int, float, or array): Minimum values below which received commands will be clipped
        input_max (int, float, or array): Maximum values above which received commands will be clipped
        output_min (int, float, or array): Lower end of range that received commands will be mapped to
        output_max (int, float, or array): Upper end of range that received commands will be mapped to
        control_min (int, float, or array): Minimum control values below which outputted controls will be clipped
        control_max (int, float, or array): Maximum control values above which outputted controls will be clipped
        control_noise (float): Amount of noise to apply. Should be in [0, 1)
        control_dim (int): Outputted control dimension -- should be number of joints from base to eef body frame
        kp (None, int, float, or array): Gain values to apply to 6DOF error.
            If None, will be variable (part of action space)
        kp_limits (2-array): (min, max) values of kp
        damping_ratio (None, int, float, or array): Damping ratio to apply to 6DOF error controller gain
            If None, will be variable (part of action space)
        damping_ratio_limits (2-array): (min, max) values of damping ratio
        kp_null (None, int, float, or array): Gain applied when calculating null torques
            If None, will be variable (part of action space)
        kp_null_limits (2-array): (min, max) values of kp_null
        rest_qpos (None, int, float, or array): If not None, sets the joint configuration used for null torques
        decouple_pos_ori (bool): Whether to decouple position and orientation control or not
    """
    def __init__(
            self,
            command_dim,
            input_min,
            input_max,
            output_min,
            output_max,
            control_min,
            control_max,
            n_dof,
            kp=150.0,
            kp_limits=(10.0, 300.),
            damping_ratio=1.0,
            damping_ratio_limits=(0.0, 2.0),
            kp_null=10.0,
            kp_null_limits=(0.0, 50.0),
            rest_qpos=None,  # init_jpos
            decouple_pos_ori=False,
            **kwargs,
    ):
        # Run super init first
        super().__init__(
            command_dim=command_dim,
            input_min=input_min,
            input_max=input_max,
            output_min=output_min,
            output_max=output_max,
            control_min=control_min,
            control_max=control_max,
            control_dim=n_dof,
        )
        # Store gains
        self.kp = self.nums2array(nums=kp, dim=6) if kp is not None else None
        self.damping_ratio = damping_ratio
        self.kd = 2 * np.sqrt(self.kp) * self.damping_ratio
        self.kp_null = self.nums2array(nums=kp_null, dim=self.control_dim) if kp_null is not None else None
        self.kd_null = 2 * np.sqrt(self.kp_null) if kp_null is not None else None  # critically damped
        self.kp_limits = np.array(kp_limits)
        self.damping_ratio_limits = np.array(damping_ratio_limits)
        self.kp_null_limits = np.array(kp_null_limits)

        # Store settings for whether we're learning gains or not
        self.variable_kp = self.kp is None
        self.variable_damping_ratio = self.damping_ratio is None
        self.variable_kp_null = self.kp_null is None

        # Modify input / output scaling based on whether we expect gains to be part of the action space
        for variable_gain, gain_limits, dim in zip(
            (self.variable_kp, self.variable_damping_ratio, self.variable_kp_null),
            (self.kp_limits, self.damping_ratio_limits, self.kp_null_limits),
            (6, 6, self.control_dim),
        ):
            if variable_gain:
                # Add this to input / output limits
                self.input_min = np.concatenate([self.input_min, self.nums2array(nums=-1., dim=dim)])
                self.input_max = np.concatenate([self.input_max, self.nums2array(nums=1., dim=dim)])
                self.output_min = np.concatenate([self.output_min, self.nums2array(nums=gain_limits[0], dim=dim)])
                self.output_max = np.concatenate([self.output_max, self.nums2array(nums=gain_limits[1], dim=dim)])
                # Update command dim
                self.command_dim += dim

        # Other values
        self.rest_qpos = self.nums2array(nums=rest_qpos, dim=self.control_dim) if rest_qpos is not None else None
        self.decouple_pos_ori = decouple_pos_ori

        self.pos_interpolator = None
        self.rot_interpolator = None

        # Initialize internal vars
        self.goal_pos = None
        self.goal_ori_mat = None
        self.last_waypoint_time = None

    def reset(self, control_dict):
        print("xxxxxxxxxxxxxxxx reset xxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        eef_pos_quat = control_dict["eef_pos_quat"]
        assert eef_pos_quat.shape == (7,)

        ee_pos = eef_pos_quat[0: 3]
        ee_quat = eef_pos_quat[3: 7]
        # Directly set goals
        self.goal_pos = np.asarray(ee_pos)
        self.goal_ori_mat = quat2mat(ee_quat)
        self.pos_interpolator = None
        self.rot_interpolator = None
        # print(f"set goal_pos: {self.goal_pos}")
        # print(f"set goal_ori_mat: {self.goal_ori_mat}")

    def update_goal(self, control_dict, command, target_time, cur_time):
        eef_pos_quat = control_dict["eef_pos_quat"]
        assert eef_pos_quat.shape == (7, ) and command.shape == (self.command_dim, )

        ee_pos = eef_pos_quat[0: 3]
        ee_quat = eef_pos_quat[3: 7]
        position_limits = control_dict.get("position_limits", None)

        # Scale the commands appropriately
        command = self.scale_command(command)
        if self.command_dim == 3:
            command = np.concatenate([command, [0.0, 0.0, 0.0]])

        delta_pose = command[:6]
        gains = command[6:]

        if self.pos_interpolator is None:
            self.pos_interpolator = LinearInterpolator(
                times=[cur_time],
                poses=[self.goal_pos],
                pose_type="eef_pos",
            )
        if self.rot_interpolator is None and self.command_dim != 3:
            self.rot_interpolator = LinearInterpolator(
                times=[cur_time],
                poses=[self.goal_ori_mat.reshape(-1)],
                pose_type="eef_ori_mat",
            )

        # Directly set goals
        self.goal_pos = np.asarray(self.goal_pos + delta_pose[:3])
        if position_limits is not None:
            assert position_limits.shape == (2, ee_pos.shape[-1])
            self.goal_pos = np.clip(self.goal_pos, position_limits[0], position_limits[1])

        self.pos_interpolator = self.pos_interpolator.schedule_waypoint(
            pose=self.goal_pos,
            target_time=target_time,
            cur_time=cur_time,
            last_waypoint_time=self.last_waypoint_time,
        )

        if self.command_dim == 3:
            pass
        else:
            self.goal_ori_mat = quat2mat(quat_mul(axisangle2quat(delta_pose[3:6]), ee_quat))
            self.rot_interpolator = self.rot_interpolator.schedule_waypoint(
                pose=self.goal_ori_mat.reshape(9),
                target_time=target_time,
                cur_time=cur_time,
                last_waypoint_time=self.last_waypoint_time,
            )
        # print(f"delta_pose: {delta_pose}, goal_pos: {self.goal_pos}")
        # print(f"last_waypoint_time: {self.last_waypoint_time}, cur_time: {cur_time}, target_time: {target_time}")
        self.last_waypoint_time = target_time
        self._update_variable_gains(gains=gains)

    def compute_control(self, control_dict, cur_time):
        if self.pos_interpolator is None:
            return None

        goal_pos = self.pos_interpolator(cur_time)
        if self.command_dim == 3:
            goal_ori_mat = self.goal_ori_mat
        else:
            goal_ori_mat = self.rot_interpolator(cur_time).reshape((3, 3))

        print(f"self.goal_pos: {self.goal_pos}, goal_pos: {goal_pos}")
        # Calculate torques
        u = _compute_osc_torques(
            control_dict=control_dict,
            goal_pos=goal_pos,
            goal_ori_mat=goal_ori_mat,
            kp=self.kp,
            kd=self.kd,
            kp_null=self.kp_null,
            kd_null=self.kd_null,
            rest_qpos=self.rest_qpos,
            control_dim=self.control_dim,
            decouple_pos_ori=self.decouple_pos_ori,
        )
        # Post-process torques (clipping + normalization)
        u = self.postprocess_control(u)
        # Return the control torques
        return u

    def _clear_variable_gains(self):
        """
        Helper function to clear any gains that we're are variable and considered part of actions
        """
        if self.variable_kp:
            self.kp = None
        if self.variable_damping_ratio:
            self.damping_ratio = None
        if self.variable_kp_null:
            self.kp_null = None
            self.kd_null = None

    def _update_variable_gains(self, gains):
        """
        Helper function to update any gains that we're are variable and considered part of actions

        Args:
            gains (tensor): (X) tensor where X dim is parsed based on which gains are being learned
        """
        assert len(gains.shape) == 1
        idx = 0

        # Ignore indexing
        if self.variable_kp:
            self.kp = gains[idx: idx+6]
            idx += 6
        if self.variable_damping_ratio:
            self.damping_ratio = gains[idx: idx+6]
            idx += 6
        if self.variable_kp_null:
            self.kp_null = gains[idx: idx+self.control_dim]
            self.kd_null = 2 * np.sqrt(self.kp_null)  # critically damped
            idx += self.control_dim

    @property
    def required_states(self):
        return ["j_eef", "mass_matrix", "gravity_torque", "position_limits", "eef_pos_quat", "eef_vel",
                "joint_pos", "joint_vel"]

    @property
    def control_type(self):
        return "joint_torque_rt"


@jit_decorator
def _compute_osc_torques(
        control_dict: Dict[str, np.ndarray],
        goal_pos: np.ndarray,
        goal_ori_mat: np.ndarray,
        kp: np.ndarray,
        kd: np.ndarray,
        kp_null: np.ndarray,
        kd_null: np.ndarray,
        rest_qpos: np.ndarray,  # init_jpos
        control_dim: int,
        decouple_pos_ori: bool,
        max_force=60,
        max_torque=60,
):
    # Extract relevant values from the control dict
    eef_pos_quat = control_dict["eef_pos_quat"]
    ee_pos = eef_pos_quat[0: 3]
    ee_quat = eef_pos_quat[3: 7]
    eef_vel = control_dict["eef_vel"]
    q = control_dict["joint_pos"]  # joint_pos, [J]
    qd = control_dict["joint_vel"]  # joint_vel, [J]
    mass_matrix = control_dict["mass_matrix"][:control_dim, :control_dim]  # [J, J]
    j_eef = control_dict["j_eef"][:, :control_dim]  # [6, J]
    gravity_torque = control_dict["gravity_torque"][:control_dim]  # [J]

    # Calculate error
    pos_err = goal_pos - ee_pos  # [3]
    vel_pos_err = -eef_vel[:3]

    ori_err = orientation_error(goal_ori_mat, quat2mat(ee_quat))  # [3]
    vel_ori_err = -eef_vel[3:]

    desired_force = pos_err * kp[0:3] + vel_pos_err * kd[0:3]
    desired_torque = ori_err * kp[3:6] + vel_ori_err * kd[3:6]

    print(f"goal_pos: {goal_pos}, ee_pos: {ee_pos}, pos_err: {pos_err}, ori_err: {ori_err}")
    print(f"goal_ori_mat: {goal_ori_mat}\nori_mat: {quat2mat(ee_quat)}")

    # Compute the inverse
    mass_matrix_inv = np.linalg.inv(mass_matrix)  # [J, J]
    m_eef_inv = j_eef @ (mass_matrix_inv @ j_eef.T)  # [6, 6]
    m_eef = np.linalg.pinv(m_eef_inv)  # [6, 6]

    # print(f"m_eef:\n{m_eef}")

    if decouple_pos_ori:
        m_eef_pos_inv = j_eef[:3, :] @ mass_matrix_inv @ j_eef[:3, :].T  # [3, 3]
        m_eef_ori_inv = j_eef[3:, :] @ mass_matrix_inv @ j_eef[3:, :].T  # [3, 3]
        m_eef_pos = np.linalg.pinv(m_eef_pos_inv)  # [3, 3]
        m_eef_ori = np.linalg.pinv(m_eef_ori_inv)  # [3, 3]
        wrench_pos = m_eef_pos @ desired_force  # [3]
        wrench_ori = m_eef_ori @ desired_torque  # [3]
    else:
        desired_wrench = np.concatenate([desired_force, desired_torque])
        wrench = m_eef @ desired_wrench  # [6]
        wrench_pos, wrench_ori = wrench[:3], wrench[3:]

    print(f"mass_matrix:\n {mass_matrix}")
    norm_abg = np.linalg.norm(wrench_ori)
    norm_xyz = np.linalg.norm(wrench_pos)
    if norm_xyz > max_force:
        wrench_pos = max_force * wrench_pos / norm_xyz

    if norm_abg > max_torque:
        wrench_ori = max_torque * wrench_ori / norm_abg

    wrench = np.concatenate([wrench_pos, wrench_ori])  # [6]

    print(f"desired_wrench: {np.concatenate([desired_force, desired_torque])}\nwrench: {wrench}, "
          f"{round(norm_xyz, 2)}, {round(norm_abg, 2)}")
    # Compute OSC torques
    torques_1 = j_eef.T @ wrench  # [J]

    # Nullspace control torques `u_null` prevents large changes in joint configuration
    # They are added into the nullspace of OSC so that the end effector orientation remains constant
    # robotics proceedings.org/rss07/p31.pdf
    torques_2 = 0
    if rest_qpos is not None:
        j_eef_inv = m_eef @ j_eef @ mass_matrix_inv  # [6, J]
        u_null = kd_null * -qd + kp_null * ((rest_qpos - q + np.pi) % (2 * np.pi) - np.pi)  # [J]
        u_null = mass_matrix @ u_null  # [J]
        torques_2 = (np.eye(control_dim) - j_eef.T @ j_eef_inv) @ u_null  # [J]
    torques = torques_1 + torques_2 + gravity_torque
    print(f"torques_1:      {torques_1}")
    print(f"torques_2:      {torques_2}")
    print(f"gravity_torque: {gravity_torque}")
    print(f"torques:        {torques}")

    return torques



