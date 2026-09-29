# Thor Deploy

从 thor1-tug 提取的独立实机部署目录，包含 G1 身体控制、Dex3 手部控制、PICO 遥操作和唯一的 0909 ONNX。无需原训练仓库，也无需安装 humanoidverse / Isaac Gym。

## 目录

- `deploy/g1_gym_deploy/`：策略推理、状态估计、遥控映射、电机监测和 LCM 类型。
- `deploy/unitree_sdk2/`：完整 SDK、DDS 库、`g1_control.cpp` 和 `hand_control.cpp`。
- `teleop/`：PICO/GMR 桥接、Dex3 手势配置及录制工具。
- `checkpoints/0909/model_10000.onnx`：20260909_232620 导出的策略；来源和 SHA256 在同目录 `manifest.json`。
- `scripts/`：编译、启动和不连接硬件的离线检查。
- `tests/`、`teleop/tests/`：部署和遥操作测试。

保留了原项目的 `deploy/` 层级。原始许可证及第三方说明保留在对应目录。

## 本机直接使用

当前机器已有 `thor1-tug-deploy`（策略）和 `gmr_axell`（遥操作）Conda 环境，可以继续使用，无需重新安装，也无需修改旧仓库的 editable install。入口会优先导入本目录的代码。

```bash
cd /home/hongwu/thor1-tug/thor-deploy
conda activate thor1-tug-deploy
bash scripts/build_sdk.sh
python scripts/check_offline.py
python -m unittest discover -s tests -v

conda activate gmr_axell
python -m unittest discover -s teleop/tests -v
```

离线检查使用真实策略、状态估计、历史观测及动作编码，LCM 替换为内存传输；不会连接或驱动机器人。分别检查 Unitree/PICO 两种来源，每种执行 100 步。需要 CUDA PyTorch，与原部署代码要求相同。

## 实机启动

先按既有实机流程关闭机器人原控制程序、确认网卡及控制权，再按顺序在独立终端启动。以下命令均从此仓库根目录执行；`eth0` 按机器人实际网卡替换。

1. 身体控制：

   ```bash
   ./deploy/unitree_sdk2/build/bin/g1_control eth0
   ```

2. 使用 Dex3 时启动手部控制：

   ```bash
   ./deploy/unitree_sdk2/build/bin/hand_control
   ```

3. 使用 PICO 时启动遥操作，`PUBLISH_DEX3_HAND=1` 启用手部输出：

   ```bash
   conda activate gmr_axell
   PUBLISH_DEX3_HAND=1 bash teleop/teleop_pose_50hz.sh
   ```

4. 启动策略，默认使用本目录的 0909 模型：

   ```bash
   conda activate thor1-tug-deploy
   RC_COMMAND_SOURCE=pico bash scripts/run_policy.sh
   ```

   使用 Unitree 遥控器则改为：

   ```bash
   RC_COMMAND_SOURCE=unitree bash scripts/run_policy.sh
   ```

   策略也支持 `--policy /path/to/model.onnx` 或 `G1_POLICY_ONNX`，优先级为命令行、环境变量、内置 0909 路径。默认模型路径相对脚本定位，移动整个目录后依然有效。

PICO 启动后先释放 grip/trigger/B；按 B 开启上肢跟随；R2 按策略终端提示校准/启动；A/X 控制右/左手开合。完整按键、断连行为和参数参见 `teleop/README.md`。手势配置为 `teleop/config/dex3_hand_poses.json`，与复制时原文件一致。运行日志由启动脚本固定写到本目录下 `logs/`。

## 换机器安装

- C++ 编译需要 CMake、C++17 编译器、LCM 开发库及 yaml-cpp 头文件。在 Ubuntu 上对应 `build-essential cmake liblcm-dev libyaml-cpp-dev`。SDK 内含 x86_64 / aarch64 库；换架构时重新编译，勿直接沿用本机二进制。
- 策略环境先安装适合目标机器的 CUDA PyTorch，再 `python -m pip install -r deploy/requirements.txt`。Jetson 使用其匹配的 PyTorch 构建。正常启动无需 `pip install -e`。
- 遥操作使用单独的 Python 3.10 环境：`python -m pip install -r teleop/requirements.txt`，再按 `teleop/README.md` 安装 GMR、支持 callback API 的 XRoboToolkit binding、PC Service 和 PICO 客户端。
- GMR、XRoboToolkit 和 Conda 环境属于机器的外部依赖，本目录不打包这些环境。本机继续使用 `/home/hongwu/teleop_ws/GMR`，它不依赖旧 thor1-tug 仓库。
- 本次验证不包含真实机器人运动、PICO 实时追踪或网络连通性；实机联调仍需连接设备完成。

验证结果见 `VALIDATION.md`。
