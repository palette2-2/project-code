# Thor Deploy

[English](README.md) | [简体中文](README.zh-CN.md)

A standalone deployment stack for **Unitree G1 (29 DoF)**, with ONNX policy
inference, Unitree remote control, optional PICO upper-body teleoperation, and
**Dex3-1** hand control. It runs independently of the training repository and does
not require Isaac Gym or `humanoidverse`.

The included baseline is the **0909 `model_10000.onnx`** policy. Training code,
training environments and other checkpoints are outside this repository.

> Publication preparation is in progress. Inherited license statements conflict,
> and the release terms for project additions and weights are not yet specified.
> See [licensing status](LICENSE.md) and [third-party notices](THIRD_PARTY_NOTICES.md).

## What is included

- A 50 Hz deployment loop: 115-dimensional observations, five-frame history
  (575 inputs), and 29 joint actions.
- C++ body and hand controllers using Unitree SDK2 and LCM.
- Selection of exactly one remote-command source: Unitree or PICO.
- PICO/GMR upper-body references, pause/resume blending and independent Dex3 toggles.
- Motor torque/temperature telemetry, regression tests and a hardware-free
  policy pipeline check.

```text
Unitree remote ── g1_control ── LCM rc_command ─┐
                                               ├─ Thor policy ── LCM ── g1_control ── G1
PICO ── XRoboToolkit ── GMR bridge ── LCM ───────┘
                              └─ hand_action ── hand_control ── Dex3
```

The policy subscribes to one RC channel; the optional upper-body reference is a
separate stream. ZMQ outputs are retained for external motion consumers but are
not needed by the Thor policy.

## Requirements

| Component | Requirement |
| --- | --- |
| Host | Linux; native build verified on Ubuntu x86_64 |
| Policy runtime | Python 3.8+, CUDA-capable GPU, CUDA-enabled PyTorch, ONNX Runtime |
| Native controllers | C++17, CMake 3.16+, pkg-config, LCM and yaml-cpp development packages |
| Optional teleoperation | Separate Python 3.10 environment, GMR, callback-enabled XRoboToolkit binding, PC Service, PICO client and trackers |
| Hardware | Compatible G1 29-DoF model; Dex3-1 when hand control is enabled |

SDK libraries for x86_64 and aarch64 are included. Rebuild on the destination
architecture; aarch64/Jetson operation was not validated in this preparation.
Install a PyTorch build appropriate for that host before installing the runtime.
ONNX Runtime may execute the actor on CPU; the surrounding deployment code still
requires CUDA PyTorch.

## Installation

Run commands from the repository root after cloning or downloading this repository.
Use an existing compatible environment or create a new one:

```bash
conda create -n thor-deploy python=3.10 -y
conda activate thor-deploy

# Install CUDA-enabled PyTorch for your platform first, then:
python -m pip install -e .
# Equivalent dependency entry: python -m pip install -r requirements.txt
```

On Ubuntu, install native build dependencies and compile:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config liblcm-dev libyaml-cpp-dev
bash scripts/build.sh
```

Outputs: `build/bin/g1_control` and `build/bin/hand_control`.
Use `BUILD_JOBS=2 bash scripts/build.sh` to limit parallel compilation.
An editable install keeps the default checkpoint relative to this checkout.
For a non-editable package install, pass `--policy` explicitly; weights are not
embedded in a Python wheel.

For optional PICO/GMR installation, follow [the teleoperation guide](teleop/README.md).
It includes the required callback patch and an import/model check. GMR, the
XRoboToolkit service and the headset client are external dependencies.

## Validate before connecting hardware

```bash
# Policy environment: software tests, then real ONNX/CUDA inference without network I/O.
bash scripts/test.sh policy
python scripts/check_offline.py

# Teleoperation environment:
bash scripts/test.sh teleop
python scripts/check_teleop.py
```

The offline check substitutes in-memory LCM, verifies the checkpoint SHA256,
and runs 100 observation/history/inference/action/message steps per RC source.
It is a software integration check, not physics simulation or proof of stability.
The teleop check loads dependencies and robot assets without starting XR streaming.
See [validation evidence](VALIDATION.md) for the tested environment and limits.

## Run on hardware

Low-level control sends motor commands. Follow the robot's control-handover
procedure, use physical support for initial checks, and keep the robot's stop
control accessible. Start with small motion commands. Software tests do not
replace checking joint mapping and motion on your hardware.

Run each component in its own terminal, from the repository root. The body
controller's interface argument must match the robot network interface.

**1. Body controller**

```bash
./build/bin/g1_control eth0
```

**2. Dex3 controller — only if using the hands**

```bash
./build/bin/hand_control
```

The hand controller immediately sends the zero/open target at startup, then holds
the last received target. A bridge disconnect does not automatically open the hands.

**3. PICO bridge — only if using PICO**

```bash
conda activate gmr_axell  # Or your teleoperation environment.
ACTUAL_HUMAN_HEIGHT=1.6 PUBLISH_DEX3_HAND=1 bash teleop/teleop_pose_50hz.sh
```

Set your actual height in metres. Omit `PUBLISH_DEX3_HAND=1` for body/arm-only use.
The launcher enables visualization; it needs a working desktop/OpenGL environment.
Keep a stable standing pose during initial height alignment.

**4. Policy**

```bash
conda activate thor-deploy  # Or your existing policy environment.
bash scripts/run_policy.sh --rc-source pico
# For the Unitree remote instead:
# bash scripts/run_policy.sh --rc-source unitree
```

The default RC source is `unitree`. `RC_COMMAND_SOURCE=pico` is also supported.
Once installed, `thor-deploy --rc-source pico` is an equivalent CLI; its logs are
relative to the current directory. The shell launcher anchors logs to this repository.
Restart the policy to change sources.

Release grip/trigger/B before operating PICO and follow the policy terminal's
R2 calibration/start prompts. Upper-body following starts paused; enable it with
B when ready. Release A/X once before using the hand toggles.

| PICO input | Function |
| --- | --- |
| Left joystick | Planar motion |
| Right joystick | Mode-dependent waist/yaw and height commands |
| Right grip | R1: stand/step toggle |
| Right trigger | R2: calibration/start/pause flow |
| B | Pause/resume upper-body following, with a 0.5 s blend on resume |
| A / X | Toggle right / left Dex3 open/closed targets |

If controller input becomes stale while the bridge is running, it zeros RC
commands and freezes upper-body/hand targets; rearming requires released buttons.
This is not an end-to-end watchdog: if the bridge or network stops entirely, the
body controller can retain the last command. Motor telemetry is monitoring, not
an automatic emergency-stop mechanism.

## Configuration

| Setting | Default / meaning |
| --- | --- |
| `--policy` / `G1_POLICY_ONNX` | CLI > environment > `checkpoints/0909/model_10000.onnx` |
| `--rc-source` / `RC_COMMAND_SOURCE` | CLI > environment > `unitree`; alternative `pico` |
| `--lcm-url` / `LCM_DEFAULT_URL` | `udpm://239.255.76.67:7667?ttl=255` |
| `ACTUAL_HUMAN_HEIGHT` | `1.6` metres; PICO launcher |
| `PUBLISH_DEX3_HAND` | `0`; set `1` to publish hand targets |
| `DEX3_POSE_CONFIG` | `teleop/config/dex3_hand_poses.json` |
| `DEX3_HAND_TRANSITION_S` | `0.5` seconds |
| `G1_SAFETY_LOG_DIR` | `logs/deploy_safety` |

Set `LCM_DEFAULT_URL` identically in all processes if changing transport. The
teleop launcher also accepts `LCM_URL`, which overrides that value locally.
Changing a channel requires updating both publisher and subscriber.
Dex3 pose JSON contains four seven-joint targets (`left_open`, `left_closed`,
`right_open`, `right_closed`), in radians. The bundled closed targets approach
joint limits; tune them for the actual hand/task after checking motion.

## Repository layout

```text
src/thor_deploy/       Python policy, environments, control helpers and LCM types
cpp/                  Project body/hand controllers and C++ LCM bindings
third_party/          Vendored Unitree SDK2 and XRoboToolkit callback patch
teleop/               Optional PICO bridge, hand configuration and recording tools
checkpoints/0909/     Single ONNX baseline and provenance/checksum manifest
scripts/              Build, launch, test and offline validation entry points
tests/                Policy/control tests (teleop tests live in teleop/tests)
docs/                 Detailed notes and retained upstream attribution
licenses/             Retained licenses; see LICENSE.md for status
```

`scripts/build_sdk.sh` remains an alias for `scripts/build.sh`. The former
`deploy/g1_gym_deploy/scripts/deploy_policy.py` path remains a compatibility
wrapper; new integrations should use the public CLI or shell launcher.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| CUDA unavailable | Run `python -c "import torch; print(torch.cuda.is_available())"` in the policy environment; check the selected PyTorch build and driver. |
| `thor_deploy` cannot be imported | Install from this root with `python -m pip install -e .`, or use `scripts/run_policy.sh`. |
| No robot telemetry / controls | Verify the body-controller NIC, multicast routing/firewall and matching transport URLs/channels. |
| Missing XR callback API | Apply the supplied binding patch and rebuild in the teleop environment; see `teleop/README.md`. |
| PICO arm/hand buttons do nothing | Release buttons to arm; check B pause state and whether hand publishing is enabled. |
| Missing native library after moving the checkout | Rebuild with `bash scripts/build.sh` on the destination host. |
| Recorder conflicts with Dex3 | The recorder also uses A/X; do not run it concurrently with hand toggles. |

## Development and attribution

See [CONTRIBUTING.md](CONTRIBUTING.md) for test commands and change conventions.
The repository includes an offline GitHub Actions workflow for tests and native
compilation; real CUDA/hardware validation remains a separate local step.

This integration builds on HOMIE, Walk These Ways, Unitree SDK2, GMR and
XRoboToolkit. Original attribution and component licenses are retained in
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). This is not an official release
of those projects. Resolve the items in [LICENSE.md](LICENSE.md) before publishing.
