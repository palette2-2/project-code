#!/usr/bin/env python3
"""Validate optional teleop dependencies and assets without starting hardware I/O."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "teleop"))


def main():
    import xrobot_teleop_to_pose_zmq_server as bridge

    bridge._load_runtime_dependencies()
    import lcm  # noqa: F401; import only, no transport
    import zmq  # noqa: F401; import only, no sockets

    model = bridge.GMR(
        src_human="xrobot",
        tgt_robot="unitree_g1",
        actual_human_height=1.6,
        verbose=False,
    )
    bridge.load_dex3_pose_config(bridge.DEFAULT_DEX3_POSE_CONFIG)
    print(f"GMR G1 model loaded: nq={model.model.nq}, nv={model.model.nv}")
    print("PASS: callback binding, GMR assets, LCM/ZMQ imports and Dex3 poses.")
    print("No streaming, robot connection or motor command was started.")


if __name__ == "__main__":
    main()
