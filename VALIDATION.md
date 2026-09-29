# Validation record

Validation date: 2026-09-29. Scope: standalone extraction and repository
organization, on the existing Ubuntu x86_64 host. No robot motion was started.

## Environment

| Component | Locally tested version |
| --- | --- |
| Policy Python | 3.8.20 |
| PyTorch | 2.4.1+cu121; CUDA available |
| ONNX Runtime | 1.19.2 |
| NumPy (policy) | 1.21.6 |
| Matplotlib / Pandas | 3.7.5 / 2.0.3 |
| Teleop Python / SciPy | 3.10.20 / 1.15.3 |
| System LCM | 1.3.1 |
| GMR source revision | `a2c0f50714376061f93b953bf38dd6e19d0c88d3` |
| XR binding source revision | `1f54475cfbcde7ed511c6ba3b1a2bba45bbdfe6f` |

The dependency ranges in `pyproject.toml` are installation requirements, not a
claim that every version/architecture combination was tested. The two existing
Conda environments were reused without replacing their runtime dependencies.

## Results after organization

- Root CMake build: `g1_control` and `hand_control` compile and link successfully.
  Their DDS dependencies resolve inside the new repository; no missing `ldd` entries.
- Policy/control tests: **28 passed**, including model-path precedence, import
  without opening LCM, CLI option forwarding and help from another directory.
- Teleop tests: **22 passed**, including PICO arming, pause/resume, Dex3
  interpolation, stale-data handling, JSON validation and LCM wire compatibility.
- Real ONNX/CUDA offline pipeline: **100 steps for each RC source**, 200 total;
  115-dimensional observations, 575-dimensional history and 29-dimensional
  outputs, with finite action/target checks and encoded LCM messages. Transport
  is in memory. Deployment imports resolve to this repository.
- Optional dependency check: callback APIs import, `xrobot -> unitree_g1` GMR
  assets and IK initialize (`nq=36`, `nv=35`), Dex3 configuration validates.
- Supplied XR callback patch: applying it to the recorded base reproduces the
  tested binding C++ source byte for byte.
- Python wheel builds using the existing build tools without fetching dependencies.
- A clean export of the staged Git tree builds both native controllers. An isolated
  temporary environment installs that export in editable mode with the required
  setuptools backend, runs the installed CLI, and passes all 50 tests, the real
  ONNX offline check and the optional teleop asset check. No untracked repository
  files are needed. External runtime dependencies were reused from the host.
- Formatting of inherited Python modules preserves their parsed syntax trees.
- The G1 whole-body checkpoint is the only ONNX; SHA256 remains
  `a3664468b58b3e02444ec1057a98a8a4fe0bf6123be8f3be13c40d1e77a339e3`.
- C++ controller source, generated LCM bindings and hand pose configuration are
  retained from the baseline. Python packaging/imports and the policy entry point
  changed; control gains, joint ordering and action scaling did not.

## Reproduce

```bash
bash scripts/build.sh
# In the policy environment:
bash scripts/test.sh policy
python scripts/check_offline.py
# In the teleop environment:
bash scripts/test.sh teleop
python scripts/check_teleop.py
```

## Limits

No actual G1/Dex3 motion, PICO live tracking, multi-host transport or Jetson/aarch64
run was tested. The offline pipeline is not a dynamics/stability test. The full
external GMR/XRoboToolkit installation was not rebuilt on a clean machine.
The included GitHub Actions workflow has not run on GitHub; its native build and
unit-test commands were exercised locally. Publication licensing remains as
recorded in `LICENSE.md`.
