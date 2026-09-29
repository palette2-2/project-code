# 2026-09-08：Pico A/X 控制 Dex3 左右手开合 V1

## 目标

在现有 PICO 全身遥操作链路中增加宇树 Dex3-1 左右手的二值开合控制：

- 右手柄 A（`right_key_one`）单击切换右手张开/闭合；
- 左手柄 X（`left_key_one`）单击切换左手张开/闭合；
- 两只手状态相互独立，长按按键不会连续切换；
- 开合目标默认在 0.5 秒内平滑过渡；
- PICO 控制数据中断时保持当前姿态，恢复后必须先松开 A/X 才能再次控制。

本版本不增加独立常驻脚本。数据链路为：

```text
PICO / XRoboToolkit
  → teleop/xrobot_teleop_to_pose_zmq_server.py
  → hand_action LCM（左手 7 维 + 右手 7 维）
  → cpp/hand_control.cpp
  → rt/dex3/left/cmd、rt/dex3/right/cmd DDS
  → Dex3-1
```

## 本次改动

### PICO bridge

`xrobot_teleop_to_pose_zmq_server.py` 增加了独立的 Dex3 状态机和
`hand_action_lcmt` 发布器。手部功能默认关闭，必须通过
`--enable_lcm_hand` 或启动脚本环境变量显式启用。

没有把 A/X 加进 `pico_rc_command`，因此原有 `rc_command_lcmt` 的 LCM
hash 和部署端兼容性不受影响。

控制状态的 ZMQ payload 增加只读信息：

```json
{
  "dex3_hand": {
    "enabled": true,
    "armed": true,
    "fresh": true,
    "left": "open",
    "right": "closed"
  }
}
```

### Dex3 接收端

`hand_control` 仍订阅 `hand_action`，但增加了以下保护：

- 启动目标在工作线程之前初始化为零位张开姿态；
- LCM 回调和 100 Hz DDS 控制循环之间通过互斥快照交换目标；
- 含 NaN/Inf 的整帧命令被拒绝；
- 每个关节在 DDS 下发前按左右手硬件范围限幅；
- LCM 数据停止后保持最后一个有效目标。

### 新增接口

| CLI 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--enable_lcm_hand` | 关闭 | 启用 Dex3 LCM 发布 |
| `--hand_lcm_channel` | `hand_action` | 手部目标 channel |
| `--dex3_pose_config` | `teleop/config/dex3_hand_poses.json` | 宇树默认开合姿态文件，可选覆盖 |
| `--hand_transition_s` | `0.5` | 开合插值时长，必须大于 0 |

`teleop_pose_50hz.sh` 对应环境变量：

| 环境变量 | 默认值 |
| --- | --- |
| `PUBLISH_DEX3_HAND` | `0` |
| `DEX3_POSE_CONFIG` | `config/dex3_hand_poses.json` |
| `DEX3_HAND_CHANNEL` | `hand_action` |
| `DEX3_HAND_TRANSITION_S` | `0.5` |

## 默认开合姿态

默认配置位于 `teleop/config/dex3_hand_poses.json`：

```json
{
  "left_open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "left_closed": [0.0, 1.05, 1.75, -1.57, -1.75, -1.57, -1.75],
  "right_open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "right_closed": [0.0, -1.05, -1.75, 1.57, 1.75, 1.57, 1.75]
}
```

张开目标使用零位。闭合目标根据
[宇树 Dex3 官方示例](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/dex3/g1_dex3_example.cpp)
给出的关节范围，沿左右手各关节的闭合方向设置到完整闭合限位；拇指第 0 关节
保持零位。配置加载时会检查字段、7 维长度、有限值和以下关节范围：

```text
left min:  [-1.05, -0.724,  0.00, -1.57, -1.75, -1.57, -1.75]
left max:  [ 1.05,  1.050,  1.75,  0.00,  0.00,  0.00,  0.00]
right min: [-1.05, -1.050, -1.75,  0.00,  0.00,  0.00,  0.00]
right max: [ 1.05,  0.742,  0.00,  1.57,  1.75,  1.57,  1.75]
```

由于 closed 目标现在位于关节限位，首次实机测试必须悬挂机器人、清空手边物体，
并确认左右手方向和硬件版本一致。若出现异常噪声、持续堵转或过大夹持力，应立即
停止 `hand_control`，并把对应 closed 目标从限位向零位回退。

## 安装与编译

在仓库根目录安装 PICO bridge 的 Python 依赖：

```bash
python -m pip install -r teleop/requirements.txt
```

编译机器人端 `hand_control`：

```bash
cmake -S . -B build
cmake --build build --target hand_control -j
```

生成的程序位于：

```text
build/bin/hand_control
```

## 部署命令

下面的路径均假定终端已经进入本仓库根目录。PC 和机器人上的 LCM multicast
URL 必须一致，并且网络需要允许 UDP multicast。

### 1. 机器人端启动 Dex3

```bash
cd build/bin
LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=255' ./hand_control
```

`hand_control` 启动后会以 100 Hz 发送零位张开目标。启动前必须保证手指周围没有
夹点或障碍物。

若修改了 PC 端的 `DEX3_HAND_CHANNEL`，机器人端需把同一个 channel 作为首个
参数，例如：

```bash
LCM_DEFAULT_URL='udpm://239.255.76.67:7667?ttl=255' ./hand_control custom_hand_action
```

### 2. 机器人端启动 G1 本体控制

另开终端，根据实机网卡选择 `eth0` 或 `eth1`：

```bash
cd build/bin
./g1_control eth0
```

### 3. PC 端启动 PICO bridge 并启用 Dex3

```bash
cd teleop
PUBLISH_DEX3_HAND=1 ./teleop_pose_50hz.sh
```

以上命令自动使用宇树默认姿态文件、`hand_action` channel 和 0.5 秒过渡时间；
无需再传 `DEX3_POSE_CONFIG`。只有需要覆盖默认值时才设置相关环境变量。

等价的直接 Python 命令是在原有启动参数中增加：

```bash
python xrobot_teleop_to_pose_zmq_server.py \
  --robot unitree_g1 \
  --enable_lcm_rc \
  --rc_lcm_channel pico_rc_command \
  --enable_lcm_hand \
  --hand_lcm_channel hand_action \
  --dex3_pose_config config/dex3_hand_poses.json \
  --hand_transition_s 0.5
```

生产部署仍推荐使用 `teleop_pose_50hz.sh`，因为它包含当前项目所需的完整
XR、ZMQ、LCM 和可视化参数。

### 4. 机器人端启动策略

```bash
bash scripts/run_policy.sh --rc-source pico
```

### 5. 开始操作

```text
1. 松开 A、X、右 Grip、右 Trigger 和 B，使各控制状态机完成解锁。
2. 按 B 启动上半身遥操作。
3. 单击 A：右手在 open/closed 之间切换。
4. 单击 X：左手在 open/closed 之间切换。
5. A/X 长按不会连续切换，两只手可以处于不同状态。
```

注意：`record_teleop_retarget_zmq.py` 当前也使用 A/X 开始和结束录制，不要在
启用 Dex3 手控时同时运行录制脚本，否则一个按键会同时触发两种功能。

## 只关闭手部功能

PICO RC 和上肢参考可以继续运行，只关闭 Dex3 LCM 发布：

```bash
cd teleop
PUBLISH_DEX3_HAND=0 ./teleop_pose_50hz.sh
```

也可以省略 `PUBLISH_DEX3_HAND`，因为其默认值为 `0`。停止 bridge 不会让正在
运行的 `hand_control` 自动张开；它会保持最后收到的目标。

## 验证流程

### 自动测试

在仓库根目录运行：

```bash
python -m unittest discover -s teleop/tests -p 'test_*.py' -v
bash scripts/test.sh policy
cmake --build build --target hand_control -j
```

测试覆盖 A/X 独立上升沿切换、release-to-arm、平滑插值、断联冻结、配置校验、
LCM wire 兼容性和发布开关。

### 实机验收

严格按以下顺序进行：

1. 不启动 `hand_control`，先确认 bridge 日志显示 `lcm_hand_enabled: true`、
   channel 和配置文件路径正确。
2. 悬挂机器人，清空手边物体后启动 `hand_control`，确认两手进入零位张开姿态。
3. 松开 A/X 后分别单击，确认左右手映射和运动方向正确。
4. 确认默认张开和抓握幅度满足任务需求；通常无需修改 JSON。
5. 在闭合、张开和运动中三个时刻分别断开 PICO，确认目标立即保持。
6. 恢复连接并保持 A/X 按下，确认手不会误动作；全部松开后再次单击才应切换。
7. 最后再进行落地站立和带物抓握测试。

## 常见问题

- A/X 没有反应：先检查启动日志中的 `lcm_hand_enabled`，然后全部松开 A/X
  至少一个控制周期，再重新单击。
- bridge 有状态但手不动：确认两端使用相同的 LCM URL 和 `hand_action` channel，
  并确认机器人端 `hand_control` 正在运行。
- 完全闭合后出现持续异响或堵转：立即停止 `hand_control`，将对应 closed 数值从
  机械限位向零位回退后再悬挂测试。
- 断联后手停在半途中：这是预期的断联冻结行为。重连、松开 A/X 后重新操作。
- 配置启动时报错：检查四个字段是否齐全、每项是否正好 7 个有限数值且未超范围。
- 多网卡环境收不到 LCM/DDS：确认 multicast 路由和宇树 DDS 使用的实机网卡配置，
  避免 VPN/TUN 抢占路由。
