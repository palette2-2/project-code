#!/usr/bin/env python3
"""Exercise the real deployment pipeline with an in-memory LCM transport."""
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'deploy/g1_gym_deploy'))

import lcm
import numpy as np
import torch


class MemoryLCM:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def subscribe(self, channel, callback):
        return (channel, callback)

    def publish(self, channel, payload):
        self.messages.append((channel, payload))


def main():
    if not torch.cuda.is_available():
        raise RuntimeError('The existing deployment pipeline requires CUDA-enabled PyTorch.')
    # Patch before importing: no multicast socket or robot connection is opened.
    with patch.object(lcm, 'LCM', MemoryLCM):
        from scripts import deploy_policy
        from envs import lcm_agent
        from envs.history_wrapper import HistoryWrapper
        from lcm_types.pd_tau_targets_lcmt import pd_tau_targets_lcmt
        from utils.cheetah_state_estimator import StateEstimator
        from utils.command_profile import RCControllerProfile
        policy_path = Path(deploy_policy.DEFAULT_POLICY_PATH)
        assert list(ROOT.rglob('*.onnx')) == [policy_path]
        manifest = json.loads((policy_path.parent / 'manifest.json').read_text())
        digest = hashlib.sha256(policy_path.read_bytes()).hexdigest()
        assert digest == manifest['sha256'], 'Checkpoint checksum mismatch'
        print(f'Checkpoint SHA256: {digest}')
        policy = deploy_policy.load_onnx_policy(str(policy_path))
        for source in ('unitree', 'pico'):
            _, channel = deploy_policy.resolve_rc_command_source(source)
            estimator = StateEstimator(MemoryLCM(), rc_command_channel=channel)
            profile = RCControllerProfile(dt=0.02, state_estimator=estimator)
            agent = lcm_agent.LCMAgent(estimator, profile)
            estimator.joint_pos = agent.default_dof_pos.copy()
            history = HistoryWrapper(agent)
            observation = history.reset()
            assert observation['obs'].shape == (1, 115)
            assert observation['obs_history'].shape == (1, 575)
            lcm_agent.lc.messages.clear()
            for step in range(100):
                action = policy(observation['obs_history'])
                assert action.shape == (1, 29)
                assert torch.isfinite(action).all()
                observation = history.step(action)
                topic, payload = lcm_agent.lc.messages[-1]
                assert topic == 'pd_plustau_targets'
                decoded = pd_tau_targets_lcmt.decode(payload)
                assert len(decoded.q_des) == 29
                assert np.isfinite(decoded.q_des).all()
            assert len(lcm_agent.lc.messages) == 100
            print(f'{source}: 100 observation/history/ONNX/action/LCM steps passed (115 -> 575 -> 29).')
        # Ensure legacy editable installs did not supply code from the old repo.
        for name, module in list(sys.modules.items()):
            if name.split('.')[0] in ('utils', 'envs', 'lcm_types', 'scripts'):
                filename = getattr(module, '__file__', None)
                if filename:
                    assert ROOT in Path(filename).resolve().parents, (name, filename)
        print('PASS: all deployment imports came from this checkout; no hardware communication.')


if __name__ == '__main__':
    main()
