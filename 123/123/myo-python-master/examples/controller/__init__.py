from functools import partial
from .base_controller import Controller
from .admittance_controller import EEAdmittanceController
from .joint_pos import JointPosController
from .osc import OSCController


CONTROLLERS = {
    "eac_pos": partial(EEAdmittanceController, command_dim=3),
    "eac": partial(EEAdmittanceController, command_dim=6),
    "joint_pos": JointPosController,
    "osc_pos": partial(OSCController, command_dim=3),
    "osc": partial(OSCController, command_dim=6),
}


DEFAULT_CONTROLLER_CONFIGS = {
    "eac_pos": {
        "input_min": [-1.0, ] * 3,
        "input_max": [1.0, ] * 3,
        "output_min": [-0.05, ] * 3,
        "output_max": [0.05, ] * 3,
        "control_min": [-0.05, -0.05, -0.05, -0.05, -0.05, -0.05],
        "control_max": [0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
    },
    "joint_pos": {
        "input_min": -1.0,
        "input_max": 1.0,
        "output_min": -3.14,
        "output_max": 3.14,
        "control_min": -3.14,
        "control_max": 3.14,
    },

    "ik": {
        "output_min": [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5],
        "output_max": [0.05, 0.05, 0.05, 0.5, 0.5, 0.5],
        "control_min": -200,
        "control_max": 200,
        "damping": 0.05,
    },
    "joint_vel": {
        "output_min": -1.0,
        "output_max": 1.0,
        "control_min": -200,
        "control_max": 200,
    },
}


def load_controller(controller_type, controller_config: dict):
    controller_class = CONTROLLERS[controller_type]
    default_controller_config = DEFAULT_CONTROLLER_CONFIGS[controller_type]

    for key, v in default_controller_config.items():
        if key not in controller_config:
            controller_config[key] = v

    return controller_class(**controller_config)


__all__ = ['Controller', 'load_controller']

