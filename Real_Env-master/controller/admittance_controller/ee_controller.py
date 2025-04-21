import math
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
from ..control_utils import jit_decorator, quat_mul, quat2mat, quat_inv, axisangle2quat
from ..base_controller import Controller
##from ...data_utils import DynamicsDataset


class EEAdmittanceController(Controller):
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
        mass_matrix (np.ndarray): Mass matrix of the 2nd order spring.
        k_matrix (np.ndarray): K matrix of the 2nd order spring.
        damp_matrix (np.ndarray): Damping matrix of the 2nd order spring.
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
            mass_matrix=None,
            k_matrix=None,
            damp_matrix=None,
            max_pos_acceleration=0.3,
            max_ori_acceleration=0.3,
            **kwargs,
    ):
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
        if mass_matrix is None:
            mp = 1
            kp = 200
            mo = 1
            ko = 15
            kp2=80
            ko2=80
            mp2=80
            mo2=80
            aux_dp=50
            aux_do=20
            mass_matrix = compute_6_by_6_diagonal_matrix_two_value(mp, mo)
            k_matrix = compute_6_by_6_diagonal_matrix_two_value(kp, ko)
            #aux_dp = critical_damping_formula(mp, kp)
            #aux_do = critical_damping_formula(mo, ko)
            damp_matrix = compute_6_by_6_diagonal_matrix_two_value(aux_dp, aux_do)

        self.mass_matrix = np.asarray(mass_matrix)  # [3, 3]
        self.inv_mass = np.linalg.inv(self.mass_matrix)  # [3, 3]
        self.k_matrix = np.asarray(k_matrix)  # [3, 3]
        self.damp_matrix = np.asarray(damp_matrix)  # [3, 3]
        self.inv_damp_matrix=np.linalg.inv(self.damp_matrix)
        self.inv_k_matrix = np.linalg.inv(self.k_matrix)
        self.max_pos_acceleration = max_pos_acceleration
        self.max_ori_acceleration = max_ori_acceleration

        self.goal_pos = None
        self.goal_ori_quat = None

        self.actual_acceleration = None
        self.actual_speed = None
        self.actual_pos_orient = None

        self.last_time = None
        self.last_ex_force = None
        self.cnt = 0
        # self.dataset = DynamicsDataset(name="ee_force")

    def reset(self, control_dict):
        eef_pos_quat = control_dict["eef_pos_quat"]
        assert eef_pos_quat.shape == (7,)

        ee_pos = eef_pos_quat[0: 3]
        ee_quat = eef_pos_quat[3: 7]

        self.last_time = time.time()
        self.cnt = None
        self.goal_pos = ee_pos.copy()  # [3]
        self.goal_ori_quat = ee_quat.copy()  # [3]

        self.actual_speed = np.zeros(6)  # [6]
        self.actual_acceleration = np.zeros(6)  # [6]

    def update_goal(self, control_dict, command, target_time, cur_time):
        print("555555555")
        if hasattr(self, "dataset"):
            self.dataset.end()
        eef_pos_quat = control_dict["eef_pos_quat"]
        #print("eef_pos_quat=",eef_pos_quat)
        assert eef_pos_quat.shape == (7, ) and command.shape == (self.command_dim, )

        ee_pos = eef_pos_quat[0: 3]
        ee_quat = eef_pos_quat[3: 7]
        position_limits = control_dict.get("position_limits", None)

        # Scale the commands appropriately
        command = self.scale_command(command)
        if self.command_dim == 3:
            command = np.concatenate([command, [0.0, 0.0, 0.0]])

        delta_pose = command[:6]

        # Directly set goals
        self.goal_pos = np.asarray(ee_pos + delta_pose[:3])
        if position_limits is not None:
            assert position_limits.shape == (2, ee_pos.shape[-1])
            self.goal_pos = np.clip(self.goal_pos, position_limits[0], position_limits[1])
        if self.command_dim == 3:
            pass
        elif self.command_dim == 6:
            self.goal_ori_quat = quat_mul(axisangle2quat(delta_pose[3:6]), ee_quat)  # [3]
        else:
            assert False
        # print(f"delta_pose: {delta_pose}, goal_pos: {self.goal_pos}")
        # print(f"last_waypoint_time: {self.last_waypoint_time}, cur_time: {cur_time}, target_time: {target_time}")
        self.last_time = cur_time
        self.cnt = 0

    def compute_control(self, control_dict, cur_time):
        print("66666666666")
        if self.cnt is None:
            return None

        if hasattr(self, "dataset"):
            self.dataset.put(control_dict)
        dt = cur_time - self.last_time
        self.last_time = cur_time
        self.cnt += 1
        eef_pos_quat = control_dict["eef_pos_quat"]
        eef_ex_force = control_dict["eef_ex_force"]
        ee_pos = eef_pos_quat[0: 3]
        ee_quat = eef_pos_quat[3: 7]
        print("eef_ex_force", eef_ex_force)
        _wrench_control = np.zeros(6)
        if self.last_ex_force is not None:
            if np.linalg.norm(eef_ex_force - self.last_ex_force) > 100:
                assert False, f"last_ex_force: {self.last_ex_force}, eef_ex_force: {eef_ex_force}"
        self.last_ex_force = eef_ex_force
        ex_torque = np.linalg.norm(eef_ex_force)
        #print("ex_torque", ex_torque)
        if ex_torque < 20:
            _wrench_external = np.zeros_like(eef_ex_force)
        else:
            _wrench_external = np.asarray(eef_ex_force) - 20 * eef_ex_force / ex_torque
        _wrench_external = np.clip(_wrench_external * 3.0, -150, 150)

        pos_error = ee_pos - self.goal_pos  # [3]
        quat_rot_err = quat_mul(ee_quat, quat_inv(self.goal_ori_quat))
        quat_rot_err = quat_rot_err / (np.linalg.norm(quat_rot_err) + 1e-6)
        orient_error = R.from_quat(quat_rot_err).as_rotvec()#四元数变成轴角表示
        error = np.concatenate([pos_error, orient_error])  # [6]


        #print("control_dict[eef_vel]1",control_dict["eef_vel"])
        coupling_wrench_arm = np.dot(self.damp_matrix, self.actual_speed) + np.dot(self.k_matrix, error)  # [3] or [6]

        #print("_wrench_control,_wrench_external",_wrench_control,_wrench_external)
        actual_acceleration = np.dot(self.inv_mass, _wrench_control + _wrench_external - coupling_wrench_arm)

        p_acc_norm = np.linalg.norm(actual_acceleration[:3])
        if p_acc_norm > self.max_pos_acceleration:
            actual_acceleration[:3] = self.max_pos_acceleration * actual_acceleration[:3] / p_acc_norm

        o_acc_norm = np.linalg.norm(actual_acceleration[3:])
        if o_acc_norm > self.max_ori_acceleration:
            actual_acceleration[3:] = self.max_ori_acceleration * actual_acceleration[3:] / o_acc_norm

        self.actual_acceleration = actual_acceleration
        #print("self.actual_speed1", self.actual_speed)
        self.actual_speed = self.actual_speed + self.actual_acceleration * dt  # [6]
        #print("control_dict[eef_vel]2",control_dict["eef_vel"])
        #print("self.actual_speed2", self.actual_speed)
        #print("postprocess_control(self.actual_speed)", self.postprocess_control(self.actual_speed))
        # self.actual_speed[3:] = 0
        # print(f"================= {self.cnt} ===========================")
        # print(f"eef_ex_force: {eef_ex_force}, {ex_torque}")
        # print(f"wrench_external: {_wrench_external}")
        # print(f"ee_pos: {ee_pos}, {self.goal_pos}")
        # print(f"error: {error}")
        # print(f"coupling_wrench_arm: {coupling_wrench_arm}")
        # print(f"actual_acceleration: {self.actual_acceleration}")
        # print(f"actual_speed: {self.actual_speed}")
        return self.postprocess_control(self.actual_speed)

    @property
    def required_states(self):
        return ["position_limits", "eef_pos_quat", "eef_ex_force"]

    @property
    def control_type(self):
        return "eef_speed_rt"


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

