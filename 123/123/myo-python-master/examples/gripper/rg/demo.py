#!/usr/bin/env python3

import time
import argparse

from envs.real_env.gripper.rg.rg_client import RGClient


def run_demo():
    """Runs gripper open-close demonstration once."""
    rg = RGClient(gripper, toolchanger_ip, toolchanger_port)

    # if not rg.get_status()[0]:  # not busy
    #     print("Current hand opening width: " +
    #           str(rg.get_width_with_offset()) +
    #           " mm")
    if True:
        rg.open_gripper()     # fully opened
        while True:
            time.sleep(0.5)
            if not rg.get_status()[0]:
                break
        rg.close_gripper()    # fully closed
        while True:
            time.sleep(0.5)
            if not rg.get_status()[0]:
                break
        rg.move_gripper(800)  # move to middle point
        while True:
            time.sleep(0.5)
            if not rg.get_status()[0]:
                break

    rg.close_connection()


def get_options():
    """Returns user-specific options."""
    parser = argparse.ArgumentParser(description='Set options.')
    parser.add_argument(
        '--gripper', dest='gripper', type=str,
        default="rg2", choices=['rg2', 'rg6'],
        help='set gripper type, rg2 or rg6')
    parser.add_argument(
        '--ip', dest='ip', type=str, default="192.168.1.107",
        help='set ip address')
    parser.add_argument(
        '--port', dest='port', type=str, default="502",
        help='set port number')
    return parser.parse_args()


if __name__ == '__main__':
    import time
    args = get_options()
    gripper = args.gripper
    toolchanger_ip = args.ip
    toolchanger_port = args.port
    rg = RGClient(gripper, toolchanger_ip, 30003)
    rg.move_gripper(600)
    time.sleep(10)
    # run_demo()

