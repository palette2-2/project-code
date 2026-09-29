#!/usr/bin/env bash
set -euo pipefail
DEPLOY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$DEPLOY_ROOT/deploy/unitree_sdk2" -B "$DEPLOY_ROOT/deploy/unitree_sdk2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$DEPLOY_ROOT/deploy/unitree_sdk2/build" --target g1_control hand_control -j "${BUILD_JOBS:-2}"
