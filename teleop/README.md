# PICO / GMR teleoperation

[English overview](../README.md) | [中文使用说明](../README.zh-CN.md)

This optional bridge sends upper-body references, PICO remote commands and Dex3
hand targets to Thor Deploy over LCM. It also exposes ZMQ outputs for external
motion consumers; no external `sim2real` repository is required by Thor Deploy.

## 1. Prepare a separate environment

```bash
conda create -n gmr_axell python=3.10 -y
conda activate gmr_axell
# From the Thor Deploy repository root:
export THOR_DEPLOY_ROOT="$PWD"
python -m pip install -r teleop/requirements.txt
export TELEOP_WORKSPACE="$HOME/teleop_ws"
mkdir -p "$TELEOP_WORKSPACE"
```

A desktop/OpenGL environment is needed by the default visualization launcher.
Install the appropriate build tools, Python headers, OpenGL/EGL libraries and
`pybind11` for your host. Use the upstream projects' instructions for platform
packages; do not overwrite a working environment just to rename it.

## 2. Install GMR

The tested GMR checkout was clean at the following revision:

```bash
cd "$TELEOP_WORKSPACE"
git clone https://github.com/YanjieZe/GMR.git
cd GMR
git checkout a2c0f50714376061f93b953bf38dd6e19d0c88d3
python -m pip install -e .
```

[GMR's installation guide](https://github.com/YanjieZe/GMR) covers its external
requirements. The bridge uses `src_human="xrobot"`, `tgt_robot="unitree_g1"`;
robot XML/meshes and the corresponding IK configuration come from GMR.

## 3. Install XRoboToolkit service and headset client

Follow the upstream installation instructions for
[XRoboToolkit PC Service](https://github.com/XR-Robotics/XRoboToolkit-PC-Service)
and the [Unity/PICO client](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client).
Select a service build matching your operating system and architecture.
The headset, host and robot need the required network connectivity; LCM uses UDP
multicast between the bridge and controllers. Start the PC Service, pair/calibrate
the trackers, and connect the headset client to the host.

## 4. Build a callback-enabled Python binding

An unpatched binding may import successfully but still lack the APIs this bridge
requires. The supplied patch reproduces the tested local callback implementation.
Use a fresh dependency checkout and the recorded base revision:

```bash
cd "$TELEOP_WORKSPACE"
git clone https://github.com/Axellwppr/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind
git checkout 75cb1130ac63e76d8f7e7788049be415e8be44f2
git apply --check "$THOR_DEPLOY_ROOT/third_party/patches/xrobotoolkit-frame-callback.patch"
git apply "$THOR_DEPLOY_ROOT/third_party/patches/xrobotoolkit-frame-callback.patch"
```

Build the native SDK and copy its headers/library following the binding's
[upstream build instructions](https://github.com/Axellwppr/XRoboToolkit-PC-Service-Pybind).
The original layout uses:

```bash
# From XRoboToolkit-PC-Service-Pybind:
mkdir -p tmp
cd tmp
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK
bash build.sh
cd ../../../..
mkdir -p lib include
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
python -m pip install pybind11
python -m pip install . --no-build-isolation
```

The patch/base reconstruction was tested locally; a clean build of all external
services on a new machine was not performed. Those projects may have additional
platform-specific native-library requirements.

## 5. Check dependencies without starting streams

```bash
cd "$THOR_DEPLOY_ROOT"
python scripts/check_teleop.py
bash scripts/test.sh teleop
```

The check loads the binding, validates its three callback APIs, initializes the
GMR G1 model/IK configuration, and validates Dex3 poses. It does not initialize
XR streaming, connect to PICO, open LCM sockets or publish motor commands.

## 6. Start the bridge

After the robot-side controllers are prepared as described in the root README:

```bash
cd "$THOR_DEPLOY_ROOT"
ACTUAL_HUMAN_HEIGHT=1.6 PUBLISH_DEX3_HAND=1 bash teleop/teleop_pose_50hz.sh
```

Use your actual height. The initial height alignment expects a stable standing
pose. Omit `PUBLISH_DEX3_HAND=1` when not controlling the hands. Keep the default
LCM URL/channels consistent with the robot processes.

The optional ZMQ channels are `tcp://*:28701` (requests), `tcp://*:28702`
(replies), and `tcp://*:28703` (controller status). The bridge publishes LCM at
50 Hz independently of whether a ZMQ consumer is connected.

## PICO control reference

The bridge can publish three independent LCM streams at `--ctrl_fps` (normally 50 Hz):

- `ref_upper_dof_pos_channel`: 14 absolute arm joint positions in radians, seven
  left-arm DoF followed by seven right-arm DoF.
- `pico_rc_command`: an `rc_command_lcmt` compatible with the deploy process.
- `hand_action`: an optional `hand_action_lcmt` containing seven left-hand and
  seven right-hand Dex3 position targets. This output is disabled by default.

The PICO mapping is:

| PICO input | Deploy input | Function in the current deploy code |
| --- | --- | --- |
| left joystick | left joystick | planar motion command |
| right joystick | right joystick | waist/height command |
| right grip | R1 | stand/step toggle |
| right trigger | R2 | deployment start/pause/calibration flow |
| right B (`right_key_two`) | bridge-only toggle | pause/resume upper-body teleop |
| right A (`right_key_one`) | bridge-only toggle | open/close right Dex3 hand |
| left X (`left_key_one`) | bridge-only toggle | open/close left Dex3 hand |

Both sticks use a radial deadzone of `0.1`, linearly remapped to retain the full
output range. Grip and trigger press at `0.7` and release at `0.3`. At startup and
after a controller-data interruption, release right grip, right trigger, and B
once before any of these controls are accepted. If controller data is older than
250 ms, both sticks and R1/R2 are published as zero.

Dex3 hand toggles require A and X to be released once at startup and after stale
controller input. A/X use rising edges, so holding a button does not repeatedly
toggle. Each target change is interpolated over 0.5 seconds by default. Stale
input freezes the current interpolated target, and reconnecting never resumes an
interrupted movement automatically.

### Upper-body pause state machine

Upper-body teleop starts in `paused_initial` and repeatedly publishes a 14D zero
reference, matching the zero-upper deployment pose. It does not become live just
because `teleop_pose_50hz.sh` is running.

- Press B in `paused_initial` or `paused_hold` to enter `blending`. The bridge
  blends from the currently held reference toward the latest PICO/GMR pose over
  0.5 seconds; the target continues updating during the blend.
- After the blend it enters `live` and publishes the latest GMR upper-body pose.
- Press B in `live` or `blending` to enter `paused_hold`; the exact last published
  reference is latched and repeated at 50 Hz.
- If controller data or the GMR pose is older than 250 ms in `live`/`blending`,
  the bridge automatically enters `paused_hold`. Reconnection never resumes upper
  teleop automatically: release the controls to unlock, then press B again.
- Pressing B without a fresh pose leaves the upper body paused and prints one
  notice for that button press.

GMR continues updating while paused. The next resume uses a fresh moving
target and interpolates from the held reference; check the resulting motion on
your hardware before operation. The ZMQ
controller payload keeps the compatible `controller_buttons` field and adds:

```json
{
  "upper_teleop": {
    "state": "paused_initial|blending|live|paused_hold",
    "pose_fresh": true
  },
  "dex3_hand": {
    "enabled": true,
    "armed": true,
    "fresh": true,
    "left": "open|closed",
    "right": "open|closed"
  }
}
```

### Start and select the RC source

Use this order for hardware operation:

```text
g1_control
→ hand_control (when Dex3 output is enabled)
→ teleop_pose_50hz.sh
→ RC_COMMAND_SOURCE=pico python deploy_policy.py
→ release grip/trigger/B once
→ press B to start upper-body teleop
```

`deploy_policy.py` subscribes to exactly one RC channel. Its default remains the
Unitree remote:

```bash
bash scripts/run_policy.sh --rc-source unitree
bash scripts/run_policy.sh --rc-source pico
```

Do not switch sources while the deploy process is running. Safely stop it and
restart with the other value. This prevents the Unitree and PICO publishers from
competing even when both are on the network.

`teleop_pose_50hz.sh` enables PICO RC publishing by default. Available launcher
overrides include:

```bash
export LCM_URL='udpm://239.255.76.67:7667?ttl=255'
export LCM_CHANNEL='ref_upper_dof_pos_channel'
export PICO_RC_CHANNEL='pico_rc_command'
export PUBLISH_PICO_RC=0  # disable only the PICO RC publisher
export PUBLISH_DEX3_HAND=1
export DEX3_POSE_CONFIG='config/dex3_hand_poses.json'
export DEX3_HAND_CHANNEL='hand_action'
export DEX3_HAND_TRANSITION_S=0.5
```

The bridge flags corresponding to the safety settings are
`--controller_stale_timeout_ms`, `--stick_deadzone`,
`--button_press_threshold`, `--button_release_threshold`,
`--upper_teleop_toggle_button`, `--upper_teleop_blend_s`, and
`--upper_pose_stale_timeout_ms`. `--start_upper_teleop_active` is available for
controlled testing; the safe default is paused. `--disable_lcm_ref` disables the
upper reference without disabling PICO RC.

Dex3 uses `--enable_lcm_hand`, `--hand_lcm_channel`,
`--dex3_pose_config`, and `--hand_transition_s`. The bundled pose file uses a
zero/open target and full-close joint-limit targets; perform the first motion
check with the robot suspended. Do not run
`record_teleop_retarget_zmq.py` at the same time because it also assigns actions
to A and X.

Install the extra Python dependency before starting:

```bash
conda activate gmr_axell
python -m pip install -r teleop/requirements.txt
```

Validate new mappings first with motors disabled, then with the robot suspended,
and only then standing at low stick amplitude. In particular, verify B holds the
arms, resuming after moving the hands does not jump, and disconnecting PICO zeros
the old locomotion command.
