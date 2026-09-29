#!/usr/bin/env bash
set -euo pipefail
DEPLOY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DEPLOY_ROOT"
exec "${PYTHON:-python}" "$DEPLOY_ROOT/deploy/g1_gym_deploy/scripts/deploy_policy.py" "$@"
