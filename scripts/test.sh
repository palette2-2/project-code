#!/usr/bin/env bash
set -euo pipefail
DEPLOY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$DEPLOY_ROOT"
export PYTHONPATH="$DEPLOY_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
case "${1:-all}" in
    policy) "${PYTHON:-python}" -m unittest discover -s tests -v ;;
    teleop) "${PYTHON:-python}" -m unittest discover -s teleop/tests -v ;;
    all)
        "${PYTHON:-python}" -m unittest discover -s tests -v
        "${PYTHON:-python}" -m unittest discover -s teleop/tests -v
        ;;
    *) echo "Usage: $0 [all|policy|teleop]" >&2; exit 2 ;;
esac
