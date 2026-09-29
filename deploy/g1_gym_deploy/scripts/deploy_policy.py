"""Compatibility entry point. Prefer scripts/run_policy.sh."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "src"))
from thor_deploy.policy import main

if __name__ == "__main__":
    main()
