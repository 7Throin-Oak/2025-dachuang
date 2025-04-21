from .base import RealEnv
from .robot import *
from .gripper import *


REGISTERED_ENVS = {
    "RealEnv": RealEnv,

}  # -num_blocks


REGISTERED_ROBOTS = {
    "Doosan": DoosanComponent,
}


REGISTERED_GRIPPERS = {
    "RG6": RGComponent,
}


def make(env_name, robot, gripper, camera_kwargs, robot_kwargs, gripper_kwargs, **kwargs):
    assert env_name in REGISTERED_ENVS

    robot_component = REGISTERED_ROBOTS[robot](**robot_kwargs)
    gripper_component = REGISTERED_GRIPPERS[gripper](**gripper_kwargs)

    env = REGISTERED_ENVS[env_name](
        robot_component=robot_component,
        gripper_component=gripper_component,
        camera_kwargs=camera_kwargs,
        **kwargs
    )
    return env


__all__ = ['make']