# Contributing

Keep changes focused and explain the hardware behavior they affect. The project
is being prepared for publication; licensing status is recorded in
[LICENSE.md](LICENSE.md).

## Development layout

- `src/thor_deploy/`: namespaced Python deployment runtime.
- `cpp/`: project-specific body/hand controllers and generated LCM bindings.
- `third_party/`: vendored SDK and a reproducible external binding patch.
- `teleop/`: optional PICO bridge, kept separate because it uses its own environment.
- `tests/`, `teleop/tests/`: deterministic software tests.

Install the runtime with `python -m pip install -e .` after choosing the correct
PyTorch build for your host. Use `bash scripts/test.sh policy` in the policy
environment and `bash scripts/test.sh teleop` in the teleop environment.
`bash scripts/build.sh` builds the native controllers.
`python scripts/check_offline.py` additionally requires CUDA and the checkpoint.

Do not modify generated LCM messages independently on only one side. Update the
schema and all affected Python/C++ bindings together, and run wire-compatibility
tests. Preserve robot joint order, action scaling, default pose and control
frequency unless a change explicitly addresses them with validation evidence.

Describe which tests ran, environment versions, and whether validation was
software-only or on hardware. Do not claim hardware validation from the offline
check. Keep logs, recordings, local environments and build products out of Git.

New Python entry points should live under `thor_deploy`, avoid opening sockets
at import time, and work independently of the current directory. Format newly
edited Python with Black (88 columns); generated and vendor files are excluded.
