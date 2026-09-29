# Third-party notices and provenance

## Deployment lineage

The imported baseline derives from the deployment code distributed with
[HOMIE](https://homietele.github.io/), which acknowledges
[Walk These Ways](https://github.com/Improbable-AI/walk-these-ways) and
[Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2).
The original authors, citation and licensing statements are preserved in
[the archived README](docs/upstream/HOMIE-README.md) and
[the original license](licenses/HOMIE-LICENSE.txt).
This project is a derivative deployment integration, not an official HOMIE release.
The exact upstream revision of the inherited deployment snapshot is not known.

Project modifications include G1 29-DoF policy integration, remote-command
mapping, upper-body reference tracking, motor telemetry, Dex3 control,
PICO integration, package organization and deployment validation tools.
Original third-party notices remain applicable; see [LICENSE.md](LICENSE.md).

## Unitree SDK2 and bundled libraries

`third_party/unitree_sdk2/` is the snapshot imported with the deployment baseline.
Its `project()` declares version 2.0.0; the exact upstream commit was not retained.
The vendor SDK headers, libraries, examples and notices are preserved. Project
controllers and their LCM types moved to `cpp/`. The vendor CMake file no longer
builds those project controllers; the root CMake file owns those targets.

- [Unitree license](third_party/unitree_sdk2/LICENSE)
- [CycloneDDS / CycloneDDS C++ licenses](third_party/unitree_sdk2/licenses/eclipse-cyclonedds/)
- [iceoryx license](third_party/unitree_sdk2/licenses/eclipse-iceoryx/)
- [RapidJSON license](third_party/unitree_sdk2/licenses/Tencent/rapidjson/LICENSE)

The SDK directory is a vendored snapshot, not a submodule. Do not regenerate or
replace it without rechecking the native build and LCM message compatibility.

## GMR (external)

[GMR](https://github.com/YanjieZe/GMR) is installed separately. The locally
verified revision is `a2c0f50714376061f93b953bf38dd6e19d0c88d3`. Its
`xrobot -> unitree_g1` retargeter supplies the robot XML, meshes and IK configuration.
These assets are not bundled here. Follow GMR's own source and asset notices.

## XRoboToolkit (external, with a supplied callback patch)

The Python bridge requires `register_frame_callback`, `clear_frame_callback`,
and `has_frame_callback`. The tested local binding adds these interfaces in
commit `1f54475cfbcde7ed511c6ba3b1a2bba45bbdfe6f`, on top of the
[Axellwppr binding](https://github.com/Axellwppr/XRoboToolkit-PC-Service-Pybind).
Public availability of that callback commit was not established. To reproduce
it, this repository includes only the binding-source delta in
[`third_party/patches/xrobotoolkit-frame-callback.patch`](third_party/patches/xrobotoolkit-frame-callback.patch),
with the [retained MIT notice](licenses/XRoboToolkit-Pybind-LICENSE.txt).
The patch was checked against its base and reproduces the tested C++ binding
source byte for byte. Installation instructions and the full base revision are
in [teleop/README.md](teleop/README.md).

The PC Service and Unity/PICO client are separate installations from
[XR-Robotics](https://github.com/XR-Robotics). Their native libraries and services
are not redistributed here.

## Checkpoint

The single included checkpoint is **G1 Whole-Body Policy** (`policy.onnx`). Its
source run and SHA256 are in [the manifest](checkpoints/g1_whole_body/manifest.json).
This is provenance information, not a declaration of model licensing or a
hardware certification. No additional training checkpoints are bundled.
