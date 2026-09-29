import time

import lcm
import numpy as np
import torch
from utils.cheetah_state_estimator import StateEstimator
from lcm_types.pd_tau_targets_lcmt import pd_tau_targets_lcmt
from lcm_types.arm_action_lcmt import arm_action_lcmt
from utils.command_profile import RCControllerProfile
from lcm_types.body_record_lcmt import body_record_lcmt
from utils.pose_target import build_position_target
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")


class LCMAgent():
    def __init__(self, se:StateEstimator, command_profile: RCControllerProfile):
        self.se = se
        self.command_profile = command_profile

        self.dt = 1/50
        self.timestep = 0

        self.num_envs = 1
        self.num_dofs = 29
        self.num_obs = 115 # 91
        self.num_history_length = 5
        self.num_lower_dofs = 15
        self.num_commands = 8
        self.device = 'cuda:0'

        # Deployment default pose.  The upper-body entries (15:29) are zero
        # and match g1_29dof_waist_fakehand_tug_zero_upper_v1.yaml.
        self.default_dof_pos = np.array([-0.1000,  0.0000,  0.0000,  0.3000, -0.2000,  0.0000, -0.1000,  0.0000,
         0.0000,  0.3000, -0.2000,  0.0000,  0.0000,  0.0400,  0.0800,  0.0000,
         0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
         0.0000,  0.0000,  0.0000, 0.0000,  0.0000], dtype=np.float64)
        # self.p_gains = np.array([150., 150., 150., 300.,  40.,  40., 150., 150., 150., 300.,  40.,  40., 300., 200., 200., 200., 100.,  20.,  20.,  20., 200., 200., 200., 100., 20.,  20.,  20.], dtype=np.float)
        # self.d_gains = np.array([2.0000, 2.0000, 2.0000, 4.0000, 4.0000, 4.0000, 2.0000, 2.0000, 2.0000, 4.0000, 4.0000, 4.0000, 5.0000, 4.0000, 4.0000, 4.0000, 1.0000, 0.5000,0.5000, 0.5000, 4.0000, 4.0000, 4.0000, 1.0000, 0.5000, 0.5000, 0.5000], dtype=np.float)

        # self.torque_limit = np.array([ 88.,  88.,  88., 139.,  50.,  50.,  88.,  88.,  88., 139.,  50.,  50.,
        #  88.,  25.,  25.,  25.,  25.,  25.,   5.,   5.,  25.,  25.,  25.,  25.,
        #  25.,   5.,   5.])
        self.commands = np.zeros((1, self.num_commands))
        # The policy and position target are 29-DoF vectors.  The old
        # num_lower_dofs value is retained for command semantics, but must not
        # be used to size policy/calibration actions.
        self.actions = torch.zeros(self.num_dofs)
        self.last_actions = torch.zeros(self.num_dofs)
        self.gravity_vector = np.zeros(3)
        self.dof_pos = np.zeros(self.num_dofs)
        self.dof_vel = np.zeros(self.num_dofs)
        self.body_angular_vel = np.zeros(3)
        self.joint_pos_target = np.zeros(self.num_dofs)
        self.torques = np.zeros(self.num_dofs)

        self.joint_idxs = self.se.joint_idxs


    def get_obs(self):
        self.gravity_vector = self.se.get_gravity_vector()
        # self.projected_gravity = self.se.get_projected_gravity().cpu().numpy()
        # print('euler', self.se.get_rpy())

        # print("#$%#$%$#%$#", self.projected_gravity)
        # 
        cmds = np.array([[0.0000, 0.7500, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]])   #  原地不动的命令
        cmds[:, :] = self.command_profile.get_command(self.timestep * self.dt)

        # cmds[:, 4] = 1
        
        self.commands[:, :] = cmds
        # self.commands[:, :] = self.se.get_command()

        self.dof_pos = self.se.get_dof_pos()

        # body_record = body_record_lcmt()
        # body_record.q = self.dof_pos
        # lc.publish("body_data_record", body_record.encode())
        self.dof_vel = self.se.get_dof_vel()
        self.body_angular_vel = self.se.get_body_angular_vel()
        actions = self.actions.reshape(1, -1).to("cuda:0")
        # print('==========================================================================')
        # print("action: ", actions)
        # print("ang_vel: ", self.body_angular_vel * 0.25)
        # print("commands: ", self.commands[:, :] * np.array([1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
        # print("dof_pos: ", self.dof_pos - self.default_dof_pos)
        # print("dof_vel: ", self.dof_vel * 0.05)
        # print("projected_gravity: ", self.gravity_vector)
        
        
        ########  双臂张开的初始位姿
        # self.ref_upper_dof_pos = [[0.0000, 0.5000,  0.0000,  1.3000,  0.0000,  0.0000, -0.4000,  
        #                            0.0000, -0.5000, 0.0000,  1.3000,  0.0000, 0.0000,  0.4000]]  ## 一套较为稳定的ref_upper_dof_pos参数
        
        ########  跟踪avp的数据
        self.ref_upper_dof_pos = self.se.get_upper_dof_pos().reshape(1, -1)

        self.ref_upper_dof_pos = np.clip(self.ref_upper_dof_pos, -2, 2)

        ob = np.concatenate((
                            actions.cpu().detach().numpy().reshape(1, -1)[:, :],   #  29
                            self.body_angular_vel.reshape(1, -1) * 0.25,   # 3
                            self.commands[:, :] * np.array([1.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), # 8
                            (self.dof_pos - self.default_dof_pos).reshape(1, -1),   #  29
                            self.dof_vel.reshape(1, -1) * 0.05,   # 29
                            self.gravity_vector.reshape(1, -1),  # 3
                            # self.projected_gravity,   # 3
                            self.ref_upper_dof_pos   #  14
                             ), axis=1)
        
        # ############  修改   ##############
        # ob = [[-1.2484e+00, -5.4190e-01, -5.7630e-02,  1.5684e+00,  5.9251e-01,
        # -1.6871e-01, -1.1549e+00,  4.8243e-01, -2.5243e-01,  1.6376e+00,
        #  5.2027e-01,  1.7755e-01, -8.4061e-03, -4.1159e-02,  8.9282e-02,
        #  4.8587e-02,  5.2092e-02,  9.6772e-02, -1.9212e-02, -1.5383e-02,
        # -2.8959e-01, -8.9645e-02, -3.5788e-02, -1.1558e-01, -1.0259e-01,
        # -1.0435e-01, -8.6253e-03, -1.5033e-01, -2.5788e-02, -3.3787e-03,
        # -4.2734e-02, -5.1654e-02,  0.0000e+00,  1.5000e+00,  0.0000e+00,
        #  0.0000e+00,  0.0000e+00,  0.0000e+00,  0.0000e+00,  0.0000e+00,
        # -3.5232e-01, -6.6715e-02, -1.0137e-01,  4.6538e-01, -1.2711e-01,
        #  1.4704e-02, -3.5550e-01,  3.2475e-02,  3.6180e-02,  4.6928e-01,
        # -1.5213e-01, -1.2407e-02, -1.6407e-02, -5.1081e-03,  4.7380e-02,
        # -6.1196e-03,  1.6669e-01, -5.5697e-02,  1.9111e-01,  1.8700e-03,
        #  1.1077e-01, -4.2550e-02,  4.1219e-02, -2.9570e-01,  7.1991e-02,
        #  1.6287e-01, -1.6173e-03,  8.6277e-02,  4.9353e-02,  7.9441e-03,
        #  1.1645e-02, -4.4353e-03, -6.4332e-03,  2.4215e-02, -4.9975e-02,
        #  2.6537e-03,  1.7982e-02,  2.1348e-02, -8.7417e-03,  3.9171e-02,
        # -6.6008e-02,  5.1819e-03, -1.7117e-03, -5.2797e-04,  1.2211e-03,
        #  3.8594e-03,  2.4389e-03, -3.3019e-03, -2.0531e-03, -3.7204e-03,
        # -5.4390e-03,  2.9448e-03,  5.7128e-03,  4.7599e-03,  6.2453e-03,
        # -1.0385e-03,  1.0594e-02, -1.0772e-02,  2.1986e-02, -2.0771e-03,
        # -9.9976e-01, -3.3800e-02,  1.5690e-01, -7.7900e-02,  1.5910e-01,
        # -4.2000e-03,  8.6800e-02, -4.2300e-02,  2.4900e-02, -3.0240e-01,
        #  7.0500e-02,  1.7020e-01,  5.1000e-03,  9.0000e-02,  3.3900e-02]]

        # print('<<=======  ob  ========>>', ','.join(map(str, ob[0])))

        # print('@@@@@@#$%@#%$$——————gravity_vector', self.gravity_vector.reshape(1, -1))
        
        return torch.tensor(ob, device=self.device).float()

    def publish_action(self, action, hard_reset=False, init = False):
        action = action.cpu().numpy()
        command_for_robot = pd_tau_targets_lcmt()
        scaled_pos_target = build_position_target(
            action,
            self.default_dof_pos,
            self.ref_upper_dof_pos,
            init=init,
        )


        # print('shape', scaled_pos_target.shape, self.ref_upper_dof_pos.shape)
        ########  shape (1, 29) (1, 14)


        # torques = (scaled_pos_target - self.dof_pos[:12]) * self.p_gains[:12]  - self.dof_vel[:12] * self.d_gains[:12]   
        # torques = np.clip(torques[:12], -self.torque_limit[:12], self.torque_limit[:12])
        self.joint_pos_target[:29] = scaled_pos_target[:29]

        # print('^^^^^^^^^^^^^self.joint_pos_target', self.joint_pos_target[:29])

        # arm_actions = self.se.get_arm_action()
        # self.joint_pos_target[15:] = 0.#arm_actions
        # self.joint_pos_target[12] = scaled_pos_target[12] # waist
        # self.joint_pos_target[15:] = scaled_pos_target[13:]
        # self.torques[:12] = torques[:12]
        # print("torques: ", torques)
        # print("==============================================================================")
        # self.torques[15:] = torques[13:] 


        command_for_robot.q_des = self.joint_pos_target
        ##########  修改：将输出规定在初始化位置，调试机器人用   ###########
        # command_for_robot.q_des = np.array([-0.2,  0.0,   0.0,   0.6, -0.4 , 0.0 , -0.2 , 0.0  , 0.0  , 0.6 ,-0.4 , 0.0   ,0.0  , 0.0,
        #                             0.0  , 0.0  , 0.0 ,  0.0 ,  0.0  , 0.0 ,  0.0 ,  0.0 ,  0.0 ,  0.0 ,  0.0,   0.0 ,  0.0  , 0.0,
        #                             0.0])
        # print('==========================================================================')
        # print('publish')
        #############################################################


        command_for_robot.tau_ff = self.torques
        command_for_robot.timestamp_us = int(time.time() * 10 ** 6)
        # print('1111111111111111&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&')
        lc.publish("pd_plustau_targets", command_for_robot.encode())
        # print('&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&')
        # arm_action = arm_action_lcmt()
        # arm_action.act = arm_actions
        # lc.publish("new_arm_action", arm_action.encode())

    def reset(self):
        self.se.reset_command_targets()
        self.actions = torch.zeros(self.num_dofs)
        self.last_actions = torch.zeros(self.num_dofs)
        self.time = time.time()
        self.timestep = 0
        return self.get_obs()


    def step(self, actions, hard_reset=False, init = False):
        # Match the training environment's robot.control.action_clip_value.
        # The actor head is linear, so an unbounded deployment clip can turn
        # an out-of-distribution observation into a huge position target.
        clip_actions = 5.0
        self.last_actions = self.actions[:]

        # actions = actions.unsqueeze(0)
        # print('&&&&&&&&&&actions', actions, actions.size())

        self.actions = torch.clip(actions[0:1, :], -clip_actions, clip_actions)
        self.publish_action(self.actions, hard_reset=hard_reset, init = init)
        time.sleep(max(self.dt - (time.time() - self.time), 0))
        if self.timestep % 100 == 0: print(f'frq: {1 / (time.time() - self.time)} Hz');
        self.time = time.time()
        obs = self.get_obs()


        self.timestep += 1
        return obs
