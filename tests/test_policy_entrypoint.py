"""Regression tests for packaging, CLI precedence and import side effects."""

import importlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from thor_deploy import policy


class PolicyEntrypointTests(unittest.TestCase):
    def test_default_checkpoint_is_inside_checkout(self):
        root = Path(__file__).resolve().parents[1]
        self.assertEqual(
            Path(policy.resolve_policy_path(environ={})),
            root / "checkpoints/g1_whole_body/policy.onnx",
        )
        self.assertTrue(policy.DEFAULT_POLICY_PATH.is_file())

    def test_cli_overrides_environment(self):
        actual = policy.resolve_policy_path("cli.onnx", {"G1_POLICY_ONNX": "env.onnx"})
        self.assertEqual(actual, str(Path("cli.onnx").resolve()))
        actual = policy.resolve_policy_path(environ={"G1_POLICY_ONNX": "env.onnx"})
        self.assertEqual(actual, str(Path("env.onnx").resolve()))

    def test_import_does_not_open_lcm_transport(self):
        import lcm

        with patch.object(
            lcm, "LCM", side_effect=AssertionError("unexpected network access")
        ):
            importlib.reload(policy)
            importlib.reload(importlib.import_module("thor_deploy.envs.lcm_agent"))

    def test_help_works_from_an_unrelated_directory(self):
        root = Path(__file__).resolve().parents[1]
        env = dict(os.environ, PYTHONPATH=str(root / "src"))
        with tempfile.TemporaryDirectory() as cwd:
            result = subprocess.run(
                [sys.executable, "-m", "thor_deploy.policy", "--help"],
                cwd=cwd,
                env=env,
                capture_output=True,
                text=True,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--rc-source", result.stdout)

    def test_main_forwards_explicit_options(self):
        with patch.object(policy, "load_and_run_policy") as run:
            policy.main(
                [
                    "--policy",
                    "actor.onnx",
                    "--rc-source",
                    "pico",
                    "--lcm-url",
                    "memq://",
                ]
            )
        run.assert_called_once_with("actor.onnx", "pico", "memq://")


if __name__ == "__main__":
    unittest.main()
