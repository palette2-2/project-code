import glob
import hashlib
import pickle as pkl
import argparse
import lcm
import sys
from pathlib import Path

# Prefer this checkout over an editable install from a different repository.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from utils.deployment_runner import DeploymentRunner
from envs.lcm_agent import LCMAgent
from utils.cheetah_state_estimator import StateEstimator
from utils.rc_command_source import resolve_rc_command_source
from utils.command_profile import RCControllerProfile
import torch
import onnxruntime as ort

import pathlib
import os

# os.environ["LCM_DEFAULT_URL"] = "eth0"

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

DEFAULT_POLICY_PATH = str(Path(__file__).resolve().parents[3] / "checkpoints/0909/model_10000.onnx")

def get_rc_command_channel():
    return resolve_rc_command_source(os.environ.get("RC_COMMAND_SOURCE", "unitree"))


def policy_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as policy_file:
        for chunk in iter(lambda: policy_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

def resolve_policy_path(cli_policy=None, environ=None):
    """Resolve CLI, environment, then known-safe baseline policy in order."""
    environ = os.environ if environ is None else environ
    selected = cli_policy or environ.get("G1_POLICY_ONNX") or DEFAULT_POLICY_PATH
    return str(pathlib.Path(selected).expanduser().resolve())


def load_and_run_policy(policy_path=None):
    ckpt_path = resolve_policy_path(policy_path)
    if not pathlib.Path(ckpt_path).is_file():
        raise FileNotFoundError(f"Policy ONNX does not exist: {ckpt_path}")
    print(f"Policy ONNX: {ckpt_path}")
    print(f"Policy SHA256: {policy_sha256(ckpt_path)}")
    control_dt = 1/50
    rc_source, rc_channel = get_rc_command_channel()
    print(f"RC command source: {rc_source} (LCM channel: {rc_channel})")
    se = StateEstimator(lc, rc_command_channel=rc_channel, control_dt=control_dt)

    command_profile = RCControllerProfile(dt=control_dt, state_estimator=se)

    hardware_agent = LCMAgent(se, command_profile)
    se.spin()

    from envs.history_wrapper import HistoryWrapper
    hardware_agent = HistoryWrapper(hardware_agent)

    policy = load_onnx_policy(ckpt_path)

    deployment_runner = DeploymentRunner(se=None)
    deployment_runner.add_control_agent(hardware_agent, "hardware_closed_loop")
    deployment_runner.add_policy(policy)
    deployment_runner.add_command_profile(command_profile)

    max_steps = 10000000
    print(f'max steps {max_steps}')
    
    deployment_runner.run(max_steps=max_steps)


def load_onnx_policy(path):
    model = ort.InferenceSession(path)
    def run_inference(input_tensor):
        
        ########### 修改 ###########
        # print("$$$$$$$$$  input_tensor   $$$$$$$$", input_tensor.size())
        ### output:  torch.Size([1, 456])

        ort_inputs = {model.get_inputs()[0].name: input_tensor.cpu().numpy()}
        ort_outs = model.run(None, ort_inputs)

        ########### 修改 ###########
        # print("$$$$$$$$$  ort_inputs   $$$$$$$$", len(ort_inputs['input'][0]))
        ### output:  456
        # print("$$$$$$$$$  ort_outs   $$$$$$$$", len(ort_outs[0][0]))
        ### output:  12

        return torch.tensor(ort_outs[0], device="cuda:0")
    return run_inference


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Deploy a G1 ONNX policy")
    parser.add_argument(
        "--policy",
        help="ONNX policy path (overrides G1_POLICY_ONNX and the safe baseline)",
    )
    args = parser.parse_args()
    load_and_run_policy(args.policy)
