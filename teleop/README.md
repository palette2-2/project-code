# Teleop Bridge Setup Guide

This README is for a user who wants to run the live teleoperation bridge on their own Ubuntu machine.

If you follow this file from top to bottom, you should end up with:

1. a working Python environment for the teleop bridge,
2. GMR installed for live retargeting,
3. XRoboToolkit PC service installed on the host machine,
4. the XRoboToolkit Python binding installed,
5. the PICO headset configured and connected,
6. the teleop ZMQ bridge running and ready to feed `sim2real`.

## What this bridge does

The runtime in this folder does four things:

1. read live body and controller data from XRoboToolkit / PICO,
2. retarget the incoming human motion to Unitree G1 with GMR,
3. publish retargeted pose chunks over ZMQ so that `sim2real/src/motion_sources.py` can consume them,
4. publish the controlled 14-DoF upper-body reference and optional PICO remote
   commands over LCM for the `thor1-tug` deploy stack.

The included files are:

- `xrobot_teleop_to_pose_zmq_server.py`
- `default_mimic_obs.py`
- `teleop_pose_50hz.sh`
- `lcm_types/ref_upper_dof_pos_lcmt.lcm`
- `lcm_types/ref_upper_dof_pos_lcmt.py`
- `lcm_types/rc_command_lcmt.lcm`
- `lcm_types/rc_command_lcmt.py`
- `requirements.txt`

## Before you start

Use Ubuntu 22.04 / 24.04 if possible.

You need:

- this repository already checked out on your machine,
- Conda installed,
- a PICO headset with leg trackers,
- XRoboToolkit client installed on the PICO,
- XRoboToolkit PC service installed on the Ubuntu host,
- motion trackers/controllers paired and calibrated on the PICO side.

In the commands below, define one working directory for all external dependencies:

```bash
export TELEOP_WORKSPACE=$HOME/teleop_ws
mkdir -p "$TELEOP_WORKSPACE"
```

## Step 1. Create the teleop Python environment

Create a dedicated Python 3.10 environment for the teleop bridge.

```bash
conda create -n gmr_axell python=3.10 -y
conda activate gmr_axell
```

This matches TWIST2 setup, where live teleoperation and GMR run in a Python 3.10 environment separate from the low-level deployment stack.

## Step 2. Install host system dependencies

Install the build tools and runtime packages needed by GMR and the XRoboToolkit binding.

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-pip \
    libgl1 \
    libegl1 \
    libxrender1 \
    libxext6
```

Then install Conda-side helper packages used by the original setup:

```bash
conda activate gmr_axell
conda install -c conda-forge libstdcxx-ng pybind11 -y
```

## Step 3. Install GMR

Clone and install GMR into the same `gmr` environment.

```bash
cd "$TELEOP_WORKSPACE"
git clone https://github.com/YanjieZe/GMR.git
cd GMR
pip install -e .
```

The bridge in this folder also needs `pyzmq`, so install it explicitly:

```bash
pip install pyzmq
```

## Step 4. Install XRoboToolkit PC service on Ubuntu

According to the original TWIST2 instructions, you can either:

- install the Ubuntu `.deb` package from the XRoboToolkit PC service release page
    ```bash
    cd "$TELEOP_WORKSPACE"
    wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
    sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
    ```
- build the PC service from [source](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases).

After installation, start the PC service application from the Ubuntu application launcher before you start teleoperation.

## Step 5. Build and install the XRoboToolkit Python binding

The teleop bridge uses `XRobotStreamer`, and `XRobotStreamer` depends on the Python module `xrobotoolkit_sdk`.

### 5.1 Clone the binding repository

```bash
cd "$TELEOP_WORKSPACE"
git clone https://github.com/Axellwppr/XRoboToolkit-PC-Service-Pybind
cd XRoboToolkit-PC-Service-Pybind
```

### 5.2 Build the underlying XRoboToolkit native SDK

```bash
mkdir -p tmp
cd tmp
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
cd XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK
bash build.sh
cd ../../../..
```

### 5.3 Copy the built headers and shared library into the pybind repo

Run the following from inside `XRoboToolkit-PC-Service-Pybind`:

```bash
mkdir -p lib
mkdir -p include
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/PXREARobotSDK.h include/
cp -r tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/nlohmann include/nlohmann/
cp tmp/XRoboToolkit-PC-Service/RoboticsService/PXREARobotSDK/build/libPXREARobotSDK.so lib/
```

### 5.4 Install the Python module

Still inside `XRoboToolkit-PC-Service-Pybind`:

```bash
conda activate gmr_axell
pip uninstall -y xrobotoolkit_sdk
python setup.py install
```

## Step 6. Verify the Python environment

Before touching the headset, verify that the required Python modules import correctly.

```bash
conda activate gmr_axell
python - <<'PY'
import general_motion_retargeting
import xrobotoolkit_sdk
import zmq
print("general_motion_retargeting: OK")
print("xrobotoolkit_sdk: OK")
print("pyzmq: OK")
PY
```

If this fails, do not continue. Fix the environment first.

## Step 7. Install and prepare the PICO side

Install the PICO-side app from:

- `https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/`

Then prepare the headset as follows:

1. Put on the motion trackers.
2. Put the controllers on the wrists.
3. Start VR on the headset.
4. Calibrate the whole-body motion tracking.
5. Open the XRoboToolkit / XRobot app on the headset.
6. Connect the app to the IP address of your Ubuntu host.
7. Start streaming whole-body data.
8. Start streaming controller data.

The PC and the PICO headset must be able to reach each other over the network.

In practice, that means:

- they are on the same LAN, and
- the PICO can route to the Ubuntu host IP you entered in the app.
- Note: ensure the communication link from the PC to the PICO is stable and has minimal packet loss; otherwise motion jitter may occur.
- Note: if you are using `iptables/nftables/ufw` on the Ubuntu host, make sure to allow incoming connections.
- Note: if you are using `VPN/TUN` interface, stop the `VPN/TUN` while teleop, or make sure it is configured to allow the PICO to reach the host IP.
- Note: if `http_proxy` and `https_proxy` environment variables are set on the Ubuntu host, make sure have `127.0.0.1` in the `no_proxy` variable, or unset the proxy variables while teleop.

## Step 8. Verify that XR data is arriving

Once the PC service is running and the PICO is connected, verify that the Python binding can see the stream.

```bash
conda activate gmr_axell
python - <<'PY'
import xrobotoolkit_sdk as xrt

xrt.init()
print("Body data available:", xrt.is_body_data_available())
print("Headset pose:", xrt.get_headset_pose())
print("Left controller pose:", xrt.get_left_controller_pose())
print("Right controller pose:", xrt.get_right_controller_pose())
xrt.close()
PY
```

If body data is not available, the XRoboToolkit binding README suggests checking:

1. the PICO headset is connected,
2. the trackers are connected and calibrated,
3. full body tracking is enabled on the PICO side client.

## Step 9. Run the teleop bridge

Once the environment and XR data stream are ready, start the ZMQ teleop bridge from this repository.

```bash
conda activate gmr_axell
cd /home/hongwu/thor1-tug/thor-deploy/teleop
bash teleop_pose_50hz.sh
```

This starts a server that matches the `sim2real` tracking configuration:

- request socket: `tcp://*:28701`
- reply socket: `tcp://*:28702`
- controller socket: `tcp://*:28703`
- chunk size: `1` frame per reply
- control publish rate: `50 Hz`

Important: During the first few seconds after starting the script, remain in a stable standing posture. The script adjusts the z-axis offset based on foot height; if you are in another pose, the estimated z-offset may affect gait quality.

## Runtime workflow

At runtime, the teleop bridge acts as a chunked motion supplier for `sim2real`.

The high-level flow is:

1. `sim2real` maintains its own reference-motion buffer.
2. The tracking policy consumes that buffer one control step at a time.
3. When the future horizon in the buffer drops below its low-water mark, `sim2real` sends a ZMQ request asking for more frames.
4. This teleop bridge samples the latest XRoboToolkit body stream, retargets it to `unitree_g1` with GMR, and returns a small chunk of future frames.
5. `sim2real` appends those frames into its reference buffer and continues policy rollout.

More concretely:

- `sim2real` is the active side for motion fetching. It does not wait for a continuous push stream; it requests more reference frames when needed.
- The bridge exposes three ZMQ channels:
  - request channel: receives frame requests from `sim2real`
  - reply channel: sends retargeted pose chunks back
  - control channel: publishes XR controller button state
- The reply payload contains `root_pos`, `root_quat`, and `dof_pos` for each returned frame.
- On a teleop start event, `sim2real` uses the first returned frame to align the live XR reference stream to its current anchor pose, then blends into the live stream.
- During steady-state teleop, `sim2real` keeps the buffer above its waterline by repeatedly requesting new chunks before the future horizon runs out.
- The bridge may interpolate between the previously sent pose and the newest retargeted pose for non-start replies, which reduces discontinuities in the returned chunk.

## Thor1-tug PICO full-body control

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

GMR continues updating while paused, so moving the hands during a pause is safe;
the next resume uses a fresh moving target and transitions smoothly. The ZMQ
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
cd deploy/g1_gym_deploy
RC_COMMAND_SOURCE=unitree python scripts/deploy_policy.py
RC_COMMAND_SOURCE=pico python scripts/deploy_policy.py
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
pip install -r requirements.txt
```

Validate new mappings first with motors disabled, then with the robot suspended,
and only then standing at low stick amplitude. In particular, verify B holds the
arms, resuming after moving the hands does not jump, and disconnecting PICO zeros
the old locomotion command.
