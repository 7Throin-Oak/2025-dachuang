import numpy as np
from urx import ursecmon, urrtmon
import socket
import cv2
import time


class UR5RTEnvWrapper(BaseRTEnvWrapper):

    def __init__(
            self,
            wrapped_env,
            robot_ip,
            camera_id,
            max_acc,
            max_vel,
            control_freq,
            sync_freq,
            robot_joints_name: list,  # [i -> joint_name]
            gripper_joint_name: str,
    ):
        self.robot_ip = robot_ip
        self.camera_id = camera_id
        super(UR5RTEnvWrapper, self).__init__(
            wrapped_env=wrapped_env,
            max_acc=max_acc,
            max_vel=max_vel,
            control_freq=control_freq,
            sync_freq=sync_freq,
            robot_joints_name=robot_joints_name,
            gripper_joint_name=gripper_joint_name,
        )

    def initialize_robot(self) -> bool:
        self.rob_com = ursecmon.SecondaryMonitor(self.robot_ip)  # data from robot at 10Hz
        self.set_tcp((0, 0, 0.1, 0, 0, 0))
        self.set_payload(5, (0, 0, 0.1))
        time.sleep(0.2)
        self.rob_monitor = urrtmon.URRTMonitor(self.robot_ip)  # som information is only available on rt interface
        self.rob_monitor.start()

        self.rg2_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rg2_socket.connect((self.robot_ip, 30003))
        return True

    def initialize_camera(self) -> bool:
        self.real_camera = cv2.VideoCapture(self.camera_id)
        return self.real_camera.isOpened()

    def move_to_jpos(self, home_jpose):
        start_joints = np.asarray(home_jpose).tolist()
        assert len(start_joints) == 6
        while True:
            now_joints = self.rob_monitor.q_actual()
            if now_joints is not None:
                break
        now_joints = now_joints.tolist()
        assert now_joints is not None and len(now_joints) == 6
        print(f"start_joint: {start_joints}, now_joints_angle={now_joints}")
        time.sleep(0.5)
        for i in range(len(start_joints)):
            print(f"initialize the {i}st joint...")
            joints = start_joints[:i + 1] + now_joints[i + 1:]
            self.movej(joints, a=self.max_acc, v=self.max_vel)
            self.wait_robot(0.1)
        time.sleep(2)
        self.last_joints = start_joints

    def act(self, cmd: ROBOT_CMD, **kwargs):
        if cmd == ROBOT_CMD.MOVEJ:
            self.movej(q=kwargs["jpos"], a=self.max_acc, v=self.max_vel)
        elif cmd == ROBOT_CMD.SERVOJ:
            self.servoj(q=kwargs["jpos"], t=self.sync_period, lookahead_time=min(1.0, 20 * self.sync_period))
        elif cmd == ROBOT_CMD.GRIPPER_CONTROL:
            self.control_gripper(kwargs["v"])
        else:
            assert False

    def get_image(self, width=500, height=500, **kwargs):
        if not hasattr(self, "real_camera"):
            return
        ret, frame = self.real_camera.read()
        h, w, c = frame.shape
        s = min(h, w)
        hs = s // 2
        cliped_frame = frame[h//2 - hs: h//2 + hs, w//2 - hs: w//2 + hs, ::-1]
        resized_frame = cv2.resize(cliped_frame, (width, height))
        return resized_frame

    def movej(self, q, a: float, v: float, t=0, r=0):
        q = np.asarray(q).tolist()
        assert len(q) == 6, len(q)
        q = [round(i, 6) for i in q]
        prog = "movej([{},{},{},{},{},{}], a={}, v={}, t={}, r={})".format(*q, a, v, t, r)
        self.send_program(prog)

    def servoj(self, q, t=0.08, lookahead_time=0.1, gain=300):
        """

        @param q:
        @param t:
        @param lookahead_time: (0.03, 0.2)
        @param gain: (100, 2000)
        """
        q = np.asarray(q).tolist()
        assert len(q) == 6, len(q)
        q = [round(i, 6) for i in q]
        prog = "servoj([{},{},{},{},{},{}], t={}, lookahead_time={}, gain={})".format(*q, t, lookahead_time, gain)
        self.send_program(prog)

    def control_gripper(self, v):
        real_crl_v = min(255, int(v * 255. / 1.22))
        print(f"sim_crl_v={v}, real_crl_v={real_crl_v}")
        self.operate_gripper(real_crl_v)
        time.sleep(0.2)

    def is_connected(self):
        return self.rob_com.running

    def is_program_running(self):
        return self.rob_com.is_program_running()

    def send_program(self, prog):
        self.rob_com.send_program(prog)

    def set_tcp(self, tcp):
        """
        set robot flange to tool tip transformation
        """
        prog = "set_tcp(p[{}, {}, {}, {}, {}, {}])".format(*tcp)
        self.send_program(prog)

    def set_payload(self, weight, cog=None):
        """
        set payload in Kg
        cog is a vector x,y,z
        if cog is not specified, then tool center point is used
        """
        if cog:
            cog = list(cog)
            cog.insert(0, weight)
            prog = "set_payload({}, ({},{},{}))".format(*cog)
        else:
            prog = "set_payload(%s)" % weight
        self.send_program(prog)

    def operate_gripper(self, target_width):
        if not hasattr(self, "rg2_socket"):
            return
        tcp_command = "def rg2ProgOpen():\n"
        tcp_command += "\ttextmsg(\"inside RG2 function called\")\n"

        tcp_command += '\ttarget_width={}\n'.format(target_width)
        tcp_command += "\ttarget_force=40\n"
        tcp_command += "\tpayload=1.0\n"
        tcp_command += "\tset_payload1=False\n"
        tcp_command += "\tdepth_compensation=False\n"
        tcp_command += "\tslave=False\n"

        tcp_command += "\ttimeout = 0\n"
        tcp_command += "\twhile get_digital_in(9) == False:\n"
        tcp_command += "\t\ttextmsg(\"inside while\")\n"
        tcp_command += "\t\tif timeout > 400:\n"
        tcp_command += "\t\t\tbreak\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\t\ttimeout = timeout+1\n"
        tcp_command += "\t\tsync()\n"
        tcp_command += "\tend\n"
        tcp_command += "\ttextmsg(\"outside while\")\n"

        tcp_command += "\tdef bit(input):\n"
        tcp_command += "\t\tmsb=65536\n"
        tcp_command += "\t\tlocal i=0\n"
        tcp_command += "\t\tlocal output=0\n"
        tcp_command += "\t\twhile i<17:\n"
        tcp_command += "\t\t\tset_digital_out(8,True)\n"
        tcp_command += "\t\t\tif input>=msb:\n"
        tcp_command += "\t\t\t\tinput=input-msb\n"
        tcp_command += "\t\t\t\tset_digital_out(9,False)\n"
        tcp_command += "\t\t\telse:\n"
        tcp_command += "\t\t\t\tset_digital_out(9,True)\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\t\tif get_digital_in(8):\n"
        tcp_command += "\t\t\t\tout=1\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\t\tsync()\n"
        tcp_command += "\t\t\tset_digital_out(8,False)\n"
        tcp_command += "\t\t\tsync()\n"
        tcp_command += "\t\t\tinput=input*2\n"
        tcp_command += "\t\t\toutput=output*2\n"
        tcp_command += "\t\t\ti=i+1\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\t\treturn output\n"
        tcp_command += "\tend\n"
        tcp_command += "\ttextmsg(\"outside bit definition\")\n"

        tcp_command += "\ttarget_width=target_width+0.0\n"
        tcp_command += "\tif target_force>40:\n"
        tcp_command += "\t\ttarget_force=40\n"
        tcp_command += "\tend\n"

        tcp_command += "\tif target_force<4:\n"
        tcp_command += "\t\ttarget_force=4\n"
        tcp_command += "\tend\n"
        tcp_command += "\tif target_width>110:\n"
        tcp_command += "\t\ttarget_width=110\n"
        tcp_command += "\tend\n"
        tcp_command += "\tif target_width<0:\n"
        tcp_command += "\t\ttarget_width=0\n"
        tcp_command += "\tend\n"
        tcp_command += "\trg_data=floor(target_width)*4\n"
        tcp_command += "\trg_data=rg_data+floor(target_force/2)*4*111\n"
        tcp_command += "\tif slave:\n"
        tcp_command += "\t\trg_data=rg_data+16384\n"
        tcp_command += "\tend\n"

        tcp_command += "\ttextmsg(\"about to call bit\")\n"
        tcp_command += "\tbit(rg_data)\n"
        tcp_command += "\ttextmsg(\"called bit\")\n"

        tcp_command += "\tif depth_compensation:\n"
        tcp_command += "\t\tfinger_length = 55.0/1000\n"
        tcp_command += "\t\tfinger_heigth_disp = 5.0/1000\n"
        tcp_command += "\t\tcenter_displacement = 7.5/1000\n"

        tcp_command += "\t\tstart_pose = get_forward_kin()\n"
        tcp_command += "\t\tset_analog_inputrange(2, 1)\n"
        tcp_command += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
        tcp_command += "\t\tzangle = zscale*1.57079633-0.087266462\n"
        tcp_command += "\t\tzwidth = 5+110*sin(zangle)\n"

        tcp_command += "\t\tstart_depth = cos(zangle)*finger_length\n"

        tcp_command += "\t\tsync()\n"
        tcp_command += "\t\tsync()\n"
        tcp_command += "\t\ttimeout = 0\n"

        tcp_command += "\t\twhile get_digital_in(9) == True:\n"
        tcp_command += "\t\t\ttimeout=timeout+1\n"
        tcp_command += "\t\t\tsync()\n"
        tcp_command += "\t\t\tif timeout > 20:\n"
        tcp_command += "\t\t\t\tbreak\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\t\ttimeout = 0\n"
        tcp_command += "\t\twhile get_digital_in(9) == False:\n"
        tcp_command += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
        tcp_command += "\t\t\tzangle = zscale*1.57079633-0.087266462\n"
        tcp_command += "\t\t\tzwidth = 5+110*sin(zangle)\n"
        tcp_command += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n"
        tcp_command += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n"
        tcp_command += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n"
        tcp_command += "\t\t\tif timeout > 400:\n"
        tcp_command += "\t\t\t\tbreak\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\t\ttimeout=timeout+1\n"
        tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\t\tnspeed = norm(get_actual_tcp_speed())\n"
        tcp_command += "\t\twhile nspeed > 0.001:\n"
        tcp_command += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
        tcp_command += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\tend\n"
        tcp_command += "\tif depth_compensation==False:\n"
        tcp_command += "\t\ttimeout = 0\n"
        tcp_command += "\t\twhile get_digital_in(9) == True:\n"
        tcp_command += "\t\t\ttimeout = timeout+1\n"
        tcp_command += "\t\t\tsync()\n"
        tcp_command += "\t\t\tif timeout > 20:\n"
        tcp_command += "\t\t\t\tbreak\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\t\ttimeout = 0\n"
        tcp_command += "\t\twhile get_digital_in(9) == False:\n"
        tcp_command += "\t\t\ttimeout = timeout+1\n"
        tcp_command += "\t\t\tsync()\n"
        tcp_command += "\t\t\tif timeout > 400:\n"
        tcp_command += "\t\t\t\tbreak\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\tend\n"
        tcp_command += "\tif set_payload1:\n"
        tcp_command += "\t\tif slave:\n"
        tcp_command += "\t\t\tif get_analog_in(3) < 2:\n"
        tcp_command += "\t\t\t\tzslam=0\n"
        tcp_command += "\t\t\telse:\n"
        tcp_command += "\t\t\t\tzslam=payload\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\telse:\n"
        tcp_command += "\t\t\tif get_digital_in(8) == False:\n"
        tcp_command += "\t\t\t\tzmasm=0\n"
        tcp_command += "\t\t\telse:\n"
        tcp_command += "\t\t\t\tzmasm=payload\n"
        tcp_command += "\t\t\tend\n"
        tcp_command += "\t\tend\n"
        tcp_command += "\t\tzsysm=0.0\n"
        tcp_command += "\t\tzload=zmasm+zslam+zsysm\n"
        tcp_command += "\t\tset_payload(zload)\n"
        tcp_command += "\tend\n"

        tcp_command += "end\n"

        self.rg2_socket.send(str.encode(tcp_command))  # 利用字符串的encode方法编码成bytes，默认为utf-8类型
        # gripper_fully_closed = check_grasp()





