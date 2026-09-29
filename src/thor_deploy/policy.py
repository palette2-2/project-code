"""Command-line entry point and ONNX loader for the G1 deployment runtime."""

import argparse
import hashlib
import os
from pathlib import Path

from thor_deploy.utils.rc_command_source import resolve_rc_command_source

DEFAULT_POLICY_PATH = (
    Path(__file__).resolve().parents[2] / "checkpoints/0909/model_10000.onnx"
)
DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
OBSERVATION_SIZE = 115
HISTORY_LENGTH = 5
ACTION_SIZE = 29


def policy_sha256(path):
    """Return a checkpoint's digest without reading it all into memory."""
    digest = hashlib.sha256()
    with open(path, "rb") as policy_file:
        for chunk in iter(lambda: policy_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_policy_path(cli_policy=None, environ=None):
    """Resolve CLI, environment, then the bundled baseline, in that order."""
    environ = os.environ if environ is None else environ
    selected = cli_policy or environ.get("G1_POLICY_ONNX") or DEFAULT_POLICY_PATH
    return str(Path(selected).expanduser().resolve())


def load_onnx_policy(path):
    """Load a 575-input, 29-output actor; preserve the CUDA action interface."""
    import onnxruntime as ort
    import torch

    model = ort.InferenceSession(str(path))
    inputs, outputs = model.get_inputs(), model.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError("Expected one observation input and one action output")
    for tensor, width in (
        (inputs[0], OBSERVATION_SIZE * HISTORY_LENGTH),
        (outputs[0], ACTION_SIZE),
    ):
        if (
            tensor.type != "tensor(float)"
            or len(tensor.shape) != 2
            or tensor.shape[1] != width
        ):
            raise ValueError(
                f"Expected float32 [batch, {width}], got {tensor.name}: {tensor.type} {tensor.shape}"
            )

    def run_inference(input_tensor):
        ort_inputs = {inputs[0].name: input_tensor.detach().cpu().numpy()}
        ort_outs = model.run(None, ort_inputs)
        return torch.tensor(ort_outs[0], device="cuda:0")

    return run_inference


def load_and_run_policy(policy_path=None, rc_source=None, lcm_url=None):
    """Validate the model before opening transport and starting robot control."""
    import lcm
    import torch

    from thor_deploy.envs.history_wrapper import HistoryWrapper
    from thor_deploy.envs.lcm_agent import LCMAgent
    from thor_deploy.utils.cheetah_state_estimator import StateEstimator
    from thor_deploy.utils.command_profile import RCControllerProfile
    from thor_deploy.utils.deployment_runner import DeploymentRunner

    ckpt_path = resolve_policy_path(policy_path)
    if not Path(ckpt_path).is_file():
        raise FileNotFoundError(
            f"Policy ONNX does not exist: {ckpt_path}. Set --policy or G1_POLICY_ONNX."
        )
    if not torch.cuda.is_available():
        raise RuntimeError(
            "G1 deployment requires CUDA-enabled PyTorch and an available GPU."
        )
    source, channel = resolve_rc_command_source(
        rc_source or os.environ.get("RC_COMMAND_SOURCE", "unitree")
    )
    url = lcm_url or os.environ.get("LCM_DEFAULT_URL", DEFAULT_LCM_URL)
    policy = load_onnx_policy(ckpt_path)
    print(f"Policy ONNX: {ckpt_path}")
    print(f"Policy SHA256: {policy_sha256(ckpt_path)}")
    print(f"RC command source: {source} (LCM channel: {channel})")
    print(f"LCM URL: {url}")

    control_dt = 1 / 50
    se = StateEstimator(lcm.LCM(url), rc_command_channel=channel, control_dt=control_dt)
    command_profile = RCControllerProfile(dt=control_dt, state_estimator=se)
    hardware_agent = HistoryWrapper(LCMAgent(se, command_profile, lc=lcm.LCM(url)))
    se.spin()

    runner = DeploymentRunner(se=None)
    runner.add_control_agent(hardware_agent, "hardware_closed_loop")
    runner.add_policy(policy)
    runner.add_command_profile(command_profile)
    runner.run(max_steps=10_000_000)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Deploy a G1 ONNX policy with Thor Deploy"
    )
    parser.add_argument(
        "--policy",
        help="ONNX path; overrides G1_POLICY_ONNX and the bundled 0909 baseline",
    )
    parser.add_argument(
        "--rc-source",
        choices=("unitree", "pico"),
        help="RC source; overrides RC_COMMAND_SOURCE (default: unitree)",
    )
    parser.add_argument(
        "--lcm-url", help="LCM transport URL; overrides LCM_DEFAULT_URL"
    )
    args = parser.parse_args(argv)
    load_and_run_policy(args.policy, args.rc_source, args.lcm_url)


if __name__ == "__main__":
    main()
