import copy
import csv
import time
import os
from pathlib import Path

import numpy as np
import torch

########### 修改 ###########
import threading
import time
import queue
import matplotlib

matplotlib.use("Agg")  # 主线程使用非交互式后端
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.figure import Figure

import pandas as pd  # 新增：用于保存Excel
import datetime  # 新增：用于获取时间


class DeploymentRunner:
    def __init__(self, experiment_name="unnamed", se=None):
        self.agents = {}
        self.policy = None
        self.command_profile = None
        self.se = se

        self.control_agent_name = None
        self.command_agent_name = None

        ########### 修改 ###########
        # 创建线程安全的数据队列
        self.data_queue = queue.Queue(maxsize=1000)
        self.graph_running = False
        self.graph_thread = None

    def add_open_loop_agent(self, agent, name):
        self.agents[name] = agent
        self.logger.add_robot(name, agent.env.cfg)

    def add_control_agent(self, agent, name):
        self.control_agent_name = name
        self.agents[name] = agent

    def set_command_agents(self, name):
        self.command_agent = name

    def add_policy(self, policy):
        self.policy = policy

    def add_command_profile(self, command_profile):
        self.command_profile = command_profile

    def calibrate(self, wait=True):
        # first, if the robot is not in nominal pose, move slowly to the nominal pose
        for agent_name in self.agents.keys():
            if hasattr(self.agents[agent_name], "get_obs"):
                agent = self.agents[agent_name]
                agent.get_obs()
                joint_pos = agent.dof_pos

                ########  经典初始位姿（上肢全零，与训练 zero_upper 版本一致）
                # 原始部署校准目标保留作对照：
                # final_goal = np.array([-0.1000,  0.0000,  0.0000,  0.3000, -0.2000,  0.0000, -0.1000,  0.0000,
                #                         0.0000,  0.3000, -0.2000,  0.0000,  0.0000,  0.0400,  0.0800,  0.0000,
                #                         0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
                #                         0.0000,  0.0000,  0.0000, 0.0000,  0.0000], dtype=np.float)

                # Use LCMAgent's single source of truth for all 29 joints.
                final_goal = agent.default_dof_pos.copy()
                if final_goal.shape != (agent.num_dofs,):
                    raise ValueError(
                        f"Expected a ({agent.num_dofs},) calibration target, got {final_goal.shape}"
                    )
                # ########  展开双臂的初始位姿
                # final_goal = np.array([-0.1000,  0.0000,  0.0000,  0.3000, -0.2000,  0.0000, -0.1000,  0.0000,
                #                         0.0000,  0.3000, -0.2000,  0.0000,  0.0000,  0.0400,  0.0800,  0.0000,
                #                         0.5000,  0.0000,  1.3000,  0.0000,  0.0000, -0.4000,  0.0000, -0.5000,
                #                         0.0000,  1.3000,  0.0000, 0.0000,  0.4000], dtype=np.float)

                print(
                    f"About to calibrate; the robot will stand [Press R2 to calibrate]"
                )
                while wait:
                    if (
                        self.command_profile.state_estimator.right_lower_right_switch_pressed
                    ):
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = (
                            False
                        )
                        break

                target = joint_pos
                cal_action = np.zeros((agent.num_envs, agent.num_dofs))
                target_sequence = []
                while np.max(np.abs(target - final_goal)) > 0.01:
                    target -= np.clip((target - final_goal), -0.05, 0.05)
                    target_sequence += [copy.deepcopy(target)]
                for target in target_sequence:
                    next_target = target
                    action_scale = 0.25

                    # ``LCMAgent`` interprets calibration actions relative to
                    # its default pose, so convert the absolute joint target
                    # back to a default-relative action before publishing.
                    # (Using ``next_target / action_scale`` would add the
                    # default pose a second time.)
                    next_target = (next_target - agent.default_dof_pos) / action_scale
                    cal_action = next_target
                    # print("ioudiofj", torch.from_numpy(cal_action).size())
                    agent.step(torch.from_numpy(cal_action).unsqueeze(0), init=True)
                    agent.get_obs()
                    time.sleep(0.05)

                print("Starting pose calibrated [Press R2 to start controller]")
                while True:
                    if (
                        self.command_profile.state_estimator.right_lower_right_switch_pressed
                    ):
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = (
                            False
                        )
                        break

                for agent_name in self.agents.keys():
                    obs = self.agents[agent_name].reset()
                    if agent_name == self.control_agent_name:
                        control_obs = obs

        return control_obs

    def run(self, num_log_steps=1000000000, max_steps=100000000):
        assert (
            self.control_agent_name is not None
        ), "cannot deploy, runner has no control agent!"
        # assert self.policy is not None, "cannot deploy, runner has no policy!"
        assert (
            self.command_profile is not None
        ), "cannot deploy, runner has no command profile!"

        # TODO: add basic test for comms

        for agent_name in self.agents.keys():
            obs = self.agents[agent_name].reset()
            if agent_name == self.control_agent_name:
                control_obs = obs
        control_obs = self.calibrate(wait=True)["obs_history"]
        safety_dir = Path(os.environ.get("G1_SAFETY_LOG_DIR", "logs/deploy_safety"))
        safety_dir.mkdir(parents=True, exist_ok=True)
        safety_path = safety_dir / datetime.datetime.now().strftime(
            "motor_safety_%Y%m%d_%H%M%S.csv"
        )
        safety_file = safety_path.open("w", newline="")
        safety_writer = csv.writer(safety_file)
        safety_writer.writerow(
            [
                "timestamp_s",
                "step",
                "max_torque_ratio",
                "saturated_joint_count",
                "persistent_saturation",
                "max_temperature_rise_c",
                "max_ankle_temperature_rise_c",
                "faulted_joint_count",
                "action_clip_fraction",
                "cumulative_action_clip_rate",
            ]
        )
        print(f"Motor safety log: {safety_path.resolve()}")
        last_safety_warning = 0.0
        cumulative_action_clip_fraction = 0.0
        action_sample_count = 0

        # 启动图形线程
        # self.start_graph()

        # 新增：初始化数据列表，用于记录时间和rpy[0]
        data = []

        # now, run control loop
        try:
            for i in range(max_steps):

                ########### 修改 ###########
                # control_obs = torch.ones(1, 575)
                # print('setp', i)
                ###########################

                action = self.policy(control_obs)
                action_clip_fraction = float(
                    torch.mean((torch.abs(action) >= 4.999).float()).item()
                )
                cumulative_action_clip_fraction += action_clip_fraction
                action_sample_count += 1
                if i % 5 == 0:
                    estimator = self.agents[self.control_agent_name].se
                    snapshot = estimator.motor_safety_snapshot
                    if snapshot is not None:
                        safety_writer.writerow(
                            [
                                time.time(),
                                i,
                                snapshot.max_torque_ratio,
                                snapshot.saturated_joint_count,
                                int(snapshot.persistent_saturation),
                                snapshot.max_temperature_rise_c,
                                snapshot.max_ankle_temperature_rise_c,
                                snapshot.faulted_joint_count,
                                action_clip_fraction,
                                cumulative_action_clip_fraction / action_sample_count,
                            ]
                        )
                        if i % 50 == 0:
                            safety_file.flush()
                        unsafe = (
                            snapshot.persistent_saturation
                            or snapshot.max_ankle_temperature_rise_c > 15.0
                            or snapshot.faulted_joint_count > 0
                        )
                        if unsafe and time.time() - last_safety_warning >= 1.0:
                            print(
                                "WARNING motor safety: "
                                f"torque_ratio={snapshot.max_torque_ratio:.2f}, "
                                f"persistent={snapshot.persistent_saturation}, "
                                f"ankle_temp_rise="
                                f"{snapshot.max_ankle_temperature_rise_c:.1f}C, "
                                f"faults={snapshot.faulted_joint_count}"
                            )
                            last_safety_warning = time.time()

                ########### 修改 ###########
                # print('******action**********', action)
                ###########################

                ########### 修改 ###########
                # print("$$$$$$$$$  control_obs   $$$$$$$$", control_obs.size())
                ### output:  torch.Size([1, 456])

                for agent_name in self.agents.keys():
                    obs = self.agents[agent_name].step(action, init=False)

                    ########### 修改 ###########
                    # print("$$$$$$$$$  obs   $$$$$$$$", obs.keys())
                    ### output: dict_keys(['obs', 'obs_history'])
                    # print("$$$$$$$$$  obs   $$$$$$$$", obs['obs_history'].size())
                    ### output: torch.Size([1, 456])

                    if agent_name == self.control_agent_name:
                        control_obs = obs["obs_history"]

                # bad orientation emergency stop
                rpy = self.agents[self.control_agent_name].se.get_rpy()
                if abs(rpy[0]) > 1.6 or abs(rpy[1]) > 1.6:
                    self.calibrate(wait=False, low=True)

                if (
                    self.command_profile.state_estimator.right_lower_right_switch_pressed
                ):
                    control_obs = self.calibrate(wait=False)["obs_history"]
                    time.sleep(1)
                    self.command_profile.state_estimator.right_lower_right_switch_pressed = (
                        False
                    )
                    while (
                        not self.command_profile.state_estimator.right_lower_right_switch_pressed
                    ):
                        time.sleep(0.01)
                    self.command_profile.state_estimator.right_lower_right_switch_pressed = (
                        False
                    )

                # # 添加数据点到图形
                # # # print('obs', obs['obs'][0, 36])
                # try:
                # # 获取当前数据点
                #     y_value = rpy[0]  # 角度
                #     # y_value = obs['obs'][0, 36].cpu().numpy()    ## cmd_stand
                #     # y_value = obs['obs'][0, 98].cpu().numpy()   ## projected_gravitey:  98\99\100
                #     self.add_data_point(y_value)
                # except Exception as e:
                #     print(f"获取数据点错误: {e}")

                # 新增：记录当前时间和rpy[1]， 也就是机器人的前后倾角
                # current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                # data.append({'时间': current_time, 'rpy[1]': rpy[1]})

            # finally, return to the nominal pose
            control_obs = self.calibrate(wait=False)

        except KeyboardInterrupt:
            # # 新增：在异常时也尝试保存数据
            # df = pd.DataFrame(data)
            # filename = f"../../身体倾斜数据/rpy_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
            # df.to_excel(filename, index=False)
            # print(f"中断时数据已保存到文件：{filename}")
            pass
        finally:
            safety_file.close()
        # finally:
        #     # 新增：在异常时也尝试保存数据
        # df = pd.DataFrame(data)
        # filename = f"../../身体倾斜数据/rpy_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
        # df.to_excel(filename, index=False)
        # print(f"中断时数据已保存到文件：{filename}")

    ########### Code for Graph Visualization
    ########### 图形可视化代码 ###########
    def start_graph(self):
        """启动图形线程"""
        if self.graph_running:
            return

        self.graph_running = True
        self.graph_thread = threading.Thread(target=self._run_graph, daemon=True)
        self.graph_thread.start()

    def _run_graph(self):
        """图形线程主函数"""
        # 切换到GUI后端 (必须在导入plt之前)
        matplotlib.use("TkAgg")  # 或 'Qt5Agg'
        import matplotlib.pyplot as plt

        # 创建图形对象
        fig, ax = plt.subplots(figsize=(40, 20))
        (line,) = ax.plot([], [], "r", lw=8)
        ax.set_xlim(0, 10)
        ax.set_ylim(-1, 1)
        ax.set_xlabel("Time (s)", fontsize=30, fontweight="bold")
        ax.set_ylabel("Value", fontsize=30, fontweight="bold")
        ax.set_title("Real-time Data Visualization", fontsize=30, fontweight="bold")
        # 设置主刻度参数
        ax.tick_params(
            axis="both",  # 同时应用x/y轴
            which="major",  # 主刻度
            labelsize=35,  # 刻度标签字号
            length=10,  # 刻度线长度(像素)
            width=2,
        )  # 刻度线宽度(像素)

        ax.grid(True)

        # 存储数据AIST_PITCH
        xdata, ydata = [], []
        start_time = time.time()

        # 初始化函数
        def init():
            line.set_data([], [])
            return (line,)

        # 更新函数
        def update(frame):
            nonlocal xdata, ydata, start_time

            # 从队列获取所有可用数据
            new_points = []
            while not self.data_queue.empty():
                try:
                    new_points.append(self.data_queue.get_nowait())
                except queue.Empty:
                    break

            # 处理新数据点
            for t, y in new_points:
                xdata.append(t)
                ydata.append(y)

            # 只保留最近1000个点
            if len(xdata) > 5000:
                xdata = xdata[-5000:]
                ydata = ydata[-5000:]

            # 更新曲线
            if xdata:
                line.set_data(xdata, ydata)

                # 动态调整X轴范围
                last_t = xdata[-1]
                if last_t > 100:
                    ax.set_xlim(last_t - 100, last_t + 1)

            return (line,)

        # 创建动画
        ani = animation.FuncAnimation(
            fig,
            update,
            init_func=init,
            interval=20,  # 每20ms更新一次
            blit=True,
            cache_frame_data=False,
        )

        # 窗口关闭处理
        def on_close(event):
            self.graph_running = False

        fig.canvas.mpl_connect("close_event", on_close)

        plt.tight_layout()
        plt.show()  # 阻塞直到窗口关闭

    def add_data_point(self, y_value):
        """添加数据点到队列（主线程调用）"""
        if self.graph_running:
            try:
                t = time.time()
                self.data_queue.put_nowait((t, y_value))
            except queue.Full:
                # 队列满时丢弃旧数据
                try:
                    self.data_queue.get_nowait()  # 丢弃一个旧数据
                    self.data_queue.put_nowait((t, y_value))  # 添加新数据
                except queue.Empty:
                    pass
