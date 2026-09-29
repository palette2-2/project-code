import math
import select
import threading
import time

import numpy as np
import torch

from thor_deploy.lcm_types.body_control_data_lcmt import body_control_data_lcmt
from thor_deploy.lcm_types.rc_command_lcmt import rc_command_lcmt
from thor_deploy.lcm_types.state_estimator_lcmt import state_estimator_lcmt
from thor_deploy.lcm_types.arm_action_lcmt import arm_action_lcmt
from thor_deploy.lcm_types.command_lcmt import command_lcmt
from thor_deploy.lcm_types.ref_upper_dof_pos_lcmt import ref_upper_dof_pos_lcmt
from thor_deploy.lcm_types.motor_safety_state_lcmt import motor_safety_state_lcmt
from thor_deploy.utils.motor_safety_monitor import MotorSafetyMonitor
from thor_deploy.utils.rc_command_mapping import (
    BaseHeightVelocityController,
    BodyYawRateWithHeadingHoldController,
    WaistYawVelocityController,
    map_planar_velocity_stick,
)
import lcm
import os


def get_rpy_from_quaternion(q):
    w, x, y, z = q
    r = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    p = np.arcsin(2 * (w * y - z * x))
    y = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.array([r, p, y])


def get_rotation_matrix_from_rpy(rpy):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]
    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    r, p, y = rpy
    R_x = np.array(
        [[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]]
    )

    R_y = np.array(
        [[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]]
    )

    R_z = np.array(
        [[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]]
    )

    rot = np.dot(R_z, np.dot(R_y, R_x))
    return rot


class StateEstimator:
    def __init__(self, lc, rc_command_channel="rc_command", control_dt=0.02):

        # reverse legs, from cpp order to isaacgym order
        self.joint_idxs = [
            0,
            1,
            2,
            3,
            4,
            5,
            6,
            7,
            8,
            9,
            10,
            11,
            12,
            13,
            14,
            15,
            16,
            17,
            18,
            19,
            20,
            21,
            22,
            23,
            24,
            25,
            26,
            27,
            28,
        ]

        self.lc = lc
        self.rc_command_channel = str(rc_command_channel)
        # os.environ["LCM_DEFAULT_URL"] = "wlan0"
        # self.new_lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
        self.num_dofs = 29
        self.joint_pos = np.zeros(self.num_dofs)
        self.joint_vel = np.zeros(self.num_dofs)
        self.motor_tau_est = np.zeros(self.num_dofs)
        self.motor_temperature = np.zeros((self.num_dofs, 2), dtype=np.int16)
        self.motor_state = np.zeros(self.num_dofs, dtype=np.int32)
        self.motor_safety_snapshot = None
        self.motor_safety_monitor = MotorSafetyMonitor(control_dt=control_dt)
        self.arm_actions = np.zeros(14)
        self.euler = np.zeros(3)
        self.R = np.eye(3)
        self.buf_idx = 0
        self.imu_ang_vel = np.zeros(3)

        self.left_stick = [0, 0]
        self.right_stick = [0, 0]
        self.right_lower_right_switch = 0
        self.right_lower_right_switch_pressed = 0

        self.right_upper_right_switch = 0
        self.right_upper_right_switch_pressed = 0

        self.init_time = time.time()
        self.received_first_bodydate = False

        self.avp_upper_dof_pos = np.zeros(14)

        self.imu_subscription = self.lc.subscribe("state_estimator_data", self._imu_cb)
        self.bodydate_state_subscription = self.lc.subscribe(
            "body_control_data", self._bodydata_cb
        )
        self.motor_safety_subscription = self.lc.subscribe(
            "motor_safety_state", self._motor_safety_cb
        )
        self.rc_command_subscription = self.lc.subscribe(
            self.rc_command_channel, self._rc_command_cb
        )
        self.pedal_command_subscription = self.lc.subscribe(
            "pedal_command", self._pedal_command_cb
        )
        # self.arm_action_subscrition = self.new_lcm.subscribe("arm_action_lcmt", self._arm_action_cb)

        self.lc.subscribe("ref_upper_dof_pos_channel", self._receive_upper_dof_pos)

        self.body_quat = np.array([0, 0, 0, 1])
        self.smoothing_ratio = 0.2
        self.body_ang_vel = np.zeros(3)
        self.smoothing_length = 12
        self.dt_history = np.zeros((self.smoothing_length, 1))
        self.euler_prev = np.zeros(3)
        self.deuler_history = np.zeros((self.smoothing_length, 3))
        self.timeuprev = time.time()
        self.command = np.zeros(4)
        self.command[3] = 0.74

        self.waist_yaw_controller = WaistYawVelocityController(
            control_dt=control_dt,
            max_rate=0.8,
            deadzone=0.1,
            lower=-1.0,
            upper=1.0,
        )
        self.base_height_controller = BaseHeightVelocityController(
            control_dt=control_dt,
            max_rate=0.15,
            deadzone=0.1,
            lower=0.5,
            upper=0.8,
            initial_height=0.75,
        )
        self.body_yaw_controller = BodyYawRateWithHeadingHoldController(
            proportional_gain=3.0,
            deadzone=0.1,
            yaw_rate_limit=1.0,
        )

        self.stand = 0
        self.change_time = 0.0
        self.rc_forward_calibration = (
            float(os.environ.get("RC_FORWARD_MIN", -1.0)),
            float(os.environ.get("RC_FORWARD_CENTER", 0.0)),
            float(os.environ.get("RC_FORWARD_MAX", 1.0)),
        )
        self.rc_horizontal_calibration = (
            float(os.environ.get("RC_HORIZONTAL_MIN", -1.0)),
            float(os.environ.get("RC_HORIZONTAL_CENTER", 0.0)),
            float(os.environ.get("RC_HORIZONTAL_MAX", 1.0)),
        )
        self.rc_command_debug = os.environ.get("RC_COMMAND_DEBUG", "0") == "1"
        self.rc_command_debug_last_time = 0.0

    def get_gravity_vector(self):
        grav = np.dot(self.R.T, np.array([0, 0, -1]))
        return grav

    def get_projected_gravity(self):
        v = self.get_gravity_vector()
        q = self.get_base_quat()

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        v = torch.tensor(v, device=device).unsqueeze(0)

        shape = q.shape
        q_w = q[:, -1]
        q_vec = q[:, :3]
        a = v * (2.0 * q_w**2 - 1.0).unsqueeze(-1)
        b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
        c = (
            q_vec
            * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1)
            * 2.0
        )
        return a - b + c

    def get_base_quat(self):
        euler = self.get_rpy()
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        # 计算半角
        roll_half = roll / 2.0
        pitch_half = pitch / 2.0
        yaw_half = yaw / 2.0

        # 计算 sin 和 cos
        cr = np.cos(roll_half)
        sr = np.sin(roll_half)
        cp = np.cos(pitch_half)
        sp = np.sin(pitch_half)
        cy = np.cos(yaw_half)
        sy = np.sin(yaw_half)

        # 四元数分量
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # 拼接为 [x, y, z, w]
        return torch.tensor([x, y, z, w], device=device).unsqueeze(0)

    def get_rpy(self):
        return self.euler

    def get_command(self):

        # Preserve the previous stick direction: positive right-stick Y lowers
        # the robot.  Unlike body yaw, neutral input simply holds the last
        # integrated height target and does not introduce a P-control phase.
        cmd_height = self.base_height_controller.update(-self.right_stick[1])

        # print('=======================================================')
        # print('self.right_upper_right_switch', self.right_upper_right_switch)
        # print('self.right_upper_right_switch_pressed', self.right_upper_right_switch_pressed)
        # print('msg.right_lower_left_switch', self.right_upper_right_switch)
        # print('=======================================================')

        if self.right_upper_right_switch_pressed and (
            (time.time() - self.change_time) > 2.0
        ):
            self.right_upper_right_switch_pressed = False
            self.stand = 1 - self.stand
            self.change_time = time.time()

        current_waist_yaw = self.get_dof_pos()[12]
        current_body_yaw = self.get_yaw()

        if self.stand == 0:  ### 站立时
            stand_waist_yaw = self.waist_yaw_controller.update(
                -self.left_stick[0], current_waist_yaw, enabled=True
            )
            stand_waist_roll = 0.3 * self.right_stick[0]
            stand_waist_pitch = 0.4 * self.left_stick[1]
            cmd_yaw = self.body_yaw_controller.update(
                0.0, current_body_yaw, enabled=False
            )
            cmd_x = 0
            cmd_y = 0
        else:  ### 踏步时
            stand_waist_yaw = self.waist_yaw_controller.update(
                0.0, current_waist_yaw, enabled=False
            )
            stand_waist_roll = 0
            stand_waist_pitch = 0
            cmd_yaw = self.body_yaw_controller.update(
                -self.right_stick[0], current_body_yaw, enabled=True
            )
            cmd_x, cmd_y = map_planar_velocity_stick(
                self.left_stick[1],
                self.left_stick[0],
                forward_calibration=self.rc_forward_calibration,
                horizontal_calibration=self.rc_horizontal_calibration,
                deadzone=0.1,
                max_forward_speed=1.0,
                max_lateral_speed=0.5,
            )

        now = time.time()
        if self.rc_command_debug and now - self.rc_command_debug_last_time >= 0.1:
            print(
                "RC raw_left=({:.3f},{:.3f}) mapped=(x={:.3f},y={:.3f},yaw={:.3f})".format(
                    float(self.left_stick[0]),
                    float(self.left_stick[1]),
                    float(cmd_x),
                    float(cmd_y),
                    float(cmd_yaw),
                )
            )
            self.rc_command_debug_last_time = now

        #### 异常命令处理 ####
        if stand_waist_pitch < 0:
            stand_waist_pitch = 0
        # return self.command
        return np.array(
            [
                cmd_yaw,
                cmd_height,
                cmd_x,
                cmd_y,
                self.stand,
                stand_waist_yaw,
                stand_waist_roll,
                stand_waist_pitch,
            ]
        )
        # return np.array([cmd_yaw, cmd_height, cmd_x, cmd_y, 1, stand_waist_yaw, stand_waist_roll, stand_waist_pitch])

    def get_buttons(self):
        return self.right_lower_right_switch

    def get_upper_dof_pos(self):
        return self.avp_upper_dof_pos

    def get_dof_pos(self):
        return self.joint_pos[self.joint_idxs]

    def get_dof_vel(self):
        return self.joint_vel[self.joint_idxs]

    def get_yaw(self):
        return self.euler[2]

    def reset_command_targets(self):
        """Reset height and rebase yaw targets on the latest robot state."""
        self.base_height_controller.reset()
        self.waist_yaw_controller.reset(self.get_dof_pos()[12])
        self.body_yaw_controller.reset(self.get_yaw())

    def get_body_angular_vel(self):
        # self.body_ang_vel = self.smoothing_ratio * np.mean(self.deuler_history / self.dt_history, axis=0) + (1 - self.smoothing_ratio) * self.body_ang_vel
        return self.body_ang_vel

    def get_arm_action(self):
        return self.arm_actions

    def _bodydata_cb(self, channel, data):
        if not self.received_first_bodydate:
            self.received_first_bodydate = True
            print(f"First body data: {time.time() - self.init_time}")

        msg = body_control_data_lcmt.decode(data)
        self.joint_pos = np.array(msg.q)
        self.joint_vel = np.array(msg.qd)

    def _motor_safety_cb(self, channel, data):
        msg = motor_safety_state_lcmt.decode(data)
        self.motor_tau_est = np.asarray(msg.tau_est, dtype=np.float64)
        self.motor_temperature = np.asarray(msg.temperature, dtype=np.int16).reshape(
            self.num_dofs, 2
        )
        self.motor_state = np.asarray(msg.motor_state, dtype=np.int64)
        self.motor_safety_snapshot = self.motor_safety_monitor.update(
            self.motor_tau_est,
            self.motor_temperature,
            self.motor_state,
            msg.timestamp_us,
        )

    def _arm_action_cb(self, channel, data):
        msg = arm_action_lcmt.decode(data)
        self.arm_actions = np.array(msg.act)

    def _imu_cb(self, channel, data):
        msg = state_estimator_lcmt.decode(data)

        self.euler = np.array(msg.rpy)

        self.R = get_rotation_matrix_from_rpy(self.euler)
        self.body_ang_vel = np.array(msg.omegaBody)
        # self.deuler_history[self.buf_idx % self.smoothing_length, :] = msg.rpy - self.euler_prev
        # self.dt_history[self.buf_idx % self.smoothing_length] = time.time() - self.timeuprev
        self.timeuprev = time.time()
        self.buf_idx += 1
        self.euler_prev = np.array(msg.rpy)

    def _rc_command_cb(self, channel, data):

        msg = rc_command_lcmt.decode(data)

        self.right_lower_right_switch_pressed = (
            msg.right_lower_right_switch and not self.right_lower_right_switch
        ) or self.right_lower_right_switch_pressed

        self.right_upper_right_switch_pressed = (
            msg.right_lower_left_switch and not self.right_upper_right_switch
        ) or self.right_upper_right_switch_pressed

        self.right_stick = msg.right_stick
        self.left_stick = msg.left_stick
        self.right_lower_right_switch = msg.right_lower_right_switch

        self.right_upper_right_switch = msg.right_lower_left_switch

    def _pedal_command_cb(self, channel, data):
        msg = command_lcmt.decode(data)
        self.command = msg.command

    def _receive_upper_dof_pos(self, channel, data):
        msg = ref_upper_dof_pos_lcmt.decode(data)
        # print(f"Received ref_upper_dof_pos: {msg.ref_upper_dof_pos}")
        self.avp_upper_dof_pos[:] = msg.ref_upper_dof_pos[:]

    def poll(self, cb=None):
        t = time.time()
        try:
            while True:
                timeout = 0.01
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                # nrfds, nwfds, nefds = select.select([self.new_lcm.fileno()], [], [], timeout)
                if rfds:
                    # print("message received!")
                    self.lc.handle()
                    # print(f'Freq {1. / (time.time() - t)} Hz'); t = time.time()
                else:
                    continue
                # if nrfds:
                # self.new_lcm.handle()
                # else:
                #     continue

        except KeyboardInterrupt:
            pass

    def spin(self):
        self.run_thread = threading.Thread(target=self.poll, daemon=False)
        self.run_thread.start()

    def close(self):
        self.lc.unsubscribe(self.bodydate_state_subscription)
        self.lc.unsubscribe(self.motor_safety_subscription)


if __name__ == "__main__":
    import lcm

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
    se = StateEstimator(lc)
    se.poll()
