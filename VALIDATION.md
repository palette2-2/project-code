# 验证记录（2026-09-29）

验证目录：`/home/hongwu/thor1-tug/thor-deploy`。

- 在新目录从源码重新编译 `g1_control` 和 `hand_control` 成功；`ldd` 检查无缺失动态库，DDS 库解析到新目录。
- `thor1-tug-deploy` 环境：部署遥控映射和电机监测 23 项测试通过。
- `gmr_axell` 环境：PICO、上肢参考、Dex3 状态机及 LCM 兼容性 22 项测试通过。
- 从 `/tmp` 清除 `PYTHONPATH` 后运行部署入口 `--help` 成功。
- 同样从 `/tmp` 清除 `PYTHONPATH` 后运行 `scripts/check_offline.py` 成功：真实 0909 ONNX + CUDA PyTorch，分别对 Unitree 和 PICO 来源执行 100 步原部署管线（单帧 115 维 → 历史 575 维 → 输出 29 维），输出及编码目标均有限；全部部署模块来自新目录。LCM 使用内存替身，未发送机器人命令。这是软件管线检查，不是物理仿真或稳定性验证。
- GMR 与支持 callback API 的 XRoboToolkit binding 实际导入成功；`xrobot -> unitree_g1` GMR 模型、网格和 IK 配置初始化成功（nq=36，nv=35）。
- 新目录只有一个 ONNX，没有 `.pt` / `.pth` checkpoint；模型 SHA256 与源文件一致：`a3664468b58b3e02444ec1057a98a8a4fe0bf6123be8f3be13c40d1e77a339e3`。
- 身体/手部控制源码、LCMAgent、PICO 桥和 Dex3 JSON 均逐字节与源文件一致。

使用已有环境：PyTorch 2.4.1+cu121（CUDA 可用）、ONNX Runtime 1.19.2、NumPy 1.21.6、Matplotlib 3.7.5、Pandas 2.0.3；遥操作 SciPy 1.15.3。

复制后修改限于部署入口的本地导入优先级/相对模型路径、清理旧模型注释、依赖清单、录制默认输出位置、文档及新增启动/验证脚本。原训练仓库未修改。

未执行真实机器人运动、PICO 实时追踪及跨机器网络测试；上述验证不能代替实机联调。第三方环境安装位置和操作步骤见 README。
