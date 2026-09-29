# Thor Deploy

[English](README.md) | [简体中文](README.zh-CN.md)

面向 **Unitree G1（29 自由度）** 的独立部署代码，包含 ONNX 策略推理、宇树遥控器控制，以及可选的 PICO 上肢遥操作和 **Dex3-1** 灵巧手控制。运行时不依赖原训练仓库、Isaac Gym 或 `humanoidverse`。

仓库仅保留 **0909 的 `model_10000.onnx`** 基线策略，不包含训练代码、训练环境或其他 checkpoint。

> 当前处于公开发布准备阶段。继承代码的许可证声明存在差异，项目新增部分及模型的发布条款尚未指定。具体见[许可证状态](LICENSE.md)和[第三方来源说明](THIRD_PARTY_NOTICES.md)。

## 功能范围

- 50 Hz 策略部署：单帧观测 115 维，5 帧历史共 575 维，输出 29 维关节动作。
- 基于 Unitree SDK2 和 LCM 的 C++ 身体、手部控制程序。
- 宇树遥控器与 PICO 二选一作为运动命令来源。
- PICO/GMR 上肢参考、暂停/恢复插值，以及左右 Dex3 独立开合。
- 电机力矩和温度遥测、回归测试及不连接硬件的策略管线检查。

```text
宇树遥控器 ── g1_control ── LCM rc_command ─┐
                                           ├─ Thor 策略 ── LCM ── g1_control ── G1
PICO ── XRoboToolkit ── GMR bridge ── LCM ───┘
                              └─ hand_action ── hand_control ── Dex3
```

策略只订阅一个遥控命令频道，上肢参考通过独立频道接收。桥接程序保留了供外部动作消费者使用的 ZMQ 输出，Thor 策略本身不依赖这些 ZMQ 接口。

## 环境要求

| 组件 | 要求 |
| --- | --- |
| 主机 | Linux；本次已在 Ubuntu x86_64 上验证原生编译 |
| 策略运行环境 | Python 3.8+、支持 CUDA 的 GPU、CUDA 版 PyTorch、ONNX Runtime |
| C++ 控制程序 | C++17、CMake 3.16+、pkg-config、LCM 和 yaml-cpp 开发包 |
| 可选遥操作 | 独立 Python 3.10 环境、GMR、支持回调的 XRoboToolkit binding、PC Service、PICO 客户端及追踪器 |
| 机器人 | 兼容的 G1 29 自由度型号；启用手控时需要 Dex3-1 |

SDK 内含 x86_64 和 aarch64 库。换架构后需在目标机器重新编译；本次准备工作未验证 aarch64/Jetson 实机运行。安装 Python 依赖前，请先安装与目标机器匹配的 PyTorch。即使 ONNX Runtime 在 CPU 上推理，外围部署代码仍需要 CUDA PyTorch。

## 安装

克隆或下载仓库后，从仓库根目录执行以下命令。可以复用兼容环境，也可以新建：

```bash
conda create -n thor-deploy python=3.10 -y
conda activate thor-deploy

# 先安装适合本机的 CUDA 版 PyTorch，然后安装部署包：
python -m pip install -e .
# 等价依赖入口：python -m pip install -r requirements.txt
```

Ubuntu 上安装编译依赖并构建：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config liblcm-dev libyaml-cpp-dev
bash scripts/build.sh
```

产物为 `build/bin/g1_control` 和 `build/bin/hand_control`。可用 `BUILD_JOBS=2 bash scripts/build.sh` 限制编译并行数。

editable 安装会从当前仓库定位默认模型。如果采用非 editable 包安装，请通过 `--policy` 指定模型；Python wheel 不打包 checkpoint。

PICO/GMR 的可选安装见[遥操作指南](teleop/README.md)，包含必要的回调补丁及依赖、机器人模型检查。GMR、XRoboToolkit 服务和头显客户端为独立外部依赖。

## 连接硬件前验证

```bash
# 策略环境：运行软件测试，再执行不联网的真实 ONNX/CUDA 推理检查。
bash scripts/test.sh policy
python scripts/check_offline.py

# 遥操作环境：
bash scripts/test.sh teleop
python scripts/check_teleop.py
```

离线检查将 LCM 替换为内存传输，验证模型 SHA256，分别对两种遥控来源执行 100 步“观测—历史—推理—动作—消息编码”。它是软件集成检查，不是物理仿真或稳定性验证。遥操作检查只加载依赖和机器人资源，不启动 XR 数据流。已验证的环境及边界见[验证记录](VALIDATION.md)。

## 实机启动

底层控制程序会发送电机命令。请完成机器人的控制权交接流程，初次检查使用物理支撑，并确保随时能够使用机器人的停止控制。从小幅命令开始验证。软件测试不能替代实机的关节映射和运动检查。

各进程分别在独立终端、仓库根目录启动。身体控制程序的网卡参数需与机器人实际网络接口一致。

**1. 身体控制**

```bash
./build/bin/g1_control eth0
```

**2. Dex3 控制——仅在使用灵巧手时启动**

```bash
./build/bin/hand_control
```

手部控制启动后立即发送零位张开目标，之后保持最后接收的目标。桥接进程断开并不会自动让手张开。

**3. PICO 桥接——仅在使用 PICO 时启动**

```bash
conda activate gmr_axell  # 或你创建的遥操作环境。
ACTUAL_HUMAN_HEIGHT=1.6 PUBLISH_DEX3_HAND=1 bash teleop/teleop_pose_50hz.sh
```

按实际身高设置米数。只控制身体和手臂时可省略 `PUBLISH_DEX3_HAND=1`。启动脚本默认开启可视化，需要可用的桌面和 OpenGL 环境。初始身高对齐阶段保持稳定站姿。

**4. 策略推理**

```bash
conda activate thor-deploy  # 或已有的策略部署环境。
bash scripts/run_policy.sh --rc-source pico
# 使用宇树遥控器时改为：
# bash scripts/run_policy.sh --rc-source unitree
```

默认遥控来源为 `unitree`，也支持 `RC_COMMAND_SOURCE=pico`。安装后可用等价命令 `thor-deploy --rc-source pico`，但其日志相对当前工作目录写入；shell 启动脚本会固定在仓库目录运行。切换遥控来源需要重启策略进程。

操作 PICO 前先松开 grip/trigger/B，再按策略终端提示使用 R2 校准和启动。上肢跟随初始为暂停状态，准备好后用 B 开启。使用灵巧手前，A/X 也需要先释放一次。

| PICO 输入 | 功能 |
| --- | --- |
| 左摇杆 | 平面运动 |
| 右摇杆 | 随运动模式变化的腰部/偏航及高度命令 |
| 右 grip | R1：站立/踏步切换 |
| 右 trigger | R2：校准、启动、暂停流程 |
| B | 暂停/恢复上肢跟随；恢复时使用 0.5 秒插值 |
| A / X | 切换右手 / 左手张开、闭合目标 |

桥接进程仍运行但控制器输入过期时，会将遥控命令清零、冻结上肢和手部目标，恢复操作前需释放按键。这不等于端到端超时保护：如果桥接进程或网络完全中断，身体控制器可能保留最后命令。电机遥测用于监测，不是自动急停机制。

## 配置

| 配置项 | 默认值 / 含义 |
| --- | --- |
| `--policy` / `G1_POLICY_ONNX` | 命令行 > 环境变量 > `checkpoints/0909/model_10000.onnx` |
| `--rc-source` / `RC_COMMAND_SOURCE` | 命令行 > 环境变量 > `unitree`；另一选项为 `pico` |
| `--lcm-url` / `LCM_DEFAULT_URL` | `udpm://239.255.76.67:7667?ttl=255` |
| `ACTUAL_HUMAN_HEIGHT` | `1.6` 米；用于 PICO 启动脚本 |
| `PUBLISH_DEX3_HAND` | 默认 `0`，设为 `1` 发布手部目标 |
| `DEX3_POSE_CONFIG` | `teleop/config/dex3_hand_poses.json` |
| `DEX3_HAND_TRANSITION_S` | `0.5` 秒 |
| `G1_SAFETY_LOG_DIR` | `logs/deploy_safety` |

更改传输地址时，在所有进程中设置相同的 `LCM_DEFAULT_URL`。遥操作启动脚本还接受 `LCM_URL`，它会覆盖该进程的地址。更改频道时必须同步修改发布端和订阅端。

Dex3 JSON 包含 `left_open`、`left_closed`、`right_open`、`right_closed` 四个七关节目标，单位为弧度。附带的闭合姿态接近关节限位，需检查实际运动后按具体手部和任务调整。

## 目录结构

```text
src/thor_deploy/       Python 策略、环境、控制辅助模块及 LCM 类型
cpp/                  项目身体/手部控制程序及 C++ LCM 绑定
third_party/          Unitree SDK2 快照及 XRoboToolkit 回调补丁
teleop/               可选 PICO 桥接、手势配置和录制工具
checkpoints/0909/     唯一 ONNX 基线及来源/SHA256 清单
scripts/              编译、启动、测试及离线验证入口
tests/                策略/控制测试；遥操作测试位于 teleop/tests
docs/                 详细说明及保留的上游署名
licenses/             保留的许可证；整体状态见 LICENSE.md
```

`scripts/build_sdk.sh` 保留为 `scripts/build.sh` 的别名。旧路径 `deploy/g1_gym_deploy/scripts/deploy_policy.py` 保留为兼容入口，新集成建议使用 CLI 或 shell 启动脚本。

## 常见问题

| 现象 | 排查方法 |
| --- | --- |
| CUDA 不可用 | 在策略环境运行 `python -c "import torch; print(torch.cuda.is_available())"`，检查 PyTorch 构建和显卡驱动。 |
| 无法导入 `thor_deploy` | 在仓库根目录执行 `python -m pip install -e .`，或使用 `scripts/run_policy.sh`。 |
| 收不到机器人状态/控制命令 | 检查身体控制网卡、多播路由/防火墙，以及两端地址和频道。 |
| XR 缺少回调接口 | 应用随附补丁，在遥操作环境重新编译 binding；见 `teleop/README.md`。 |
| PICO 手臂/手部按键无效 | 先释放按键解锁，检查 B 暂停状态及手部发布开关。 |
| 移动目录后缺少动态库 | 在目标主机重新执行 `bash scripts/build.sh`。 |
| 录制与 Dex3 按键冲突 | 录制工具也使用 A/X，请勿与手部切换同时运行。 |

## 开发与来源

测试命令和修改约定见 [CONTRIBUTING.md](CONTRIBUTING.md)。仓库附有执行离线测试和原生编译的 GitHub Actions 工作流；真实 CUDA 和硬件验证仍需单独进行。

本集成基于 HOMIE、Walk These Ways、Unitree SDK2、GMR 和 XRoboToolkit。原署名及组件许可证保留在[第三方说明](THIRD_PARTY_NOTICES.md)中，本项目不是这些上游项目的官方发布。公开发布前需解决 [LICENSE.md](LICENSE.md) 中记录的事项。
