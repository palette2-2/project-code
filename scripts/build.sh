#!/usr/bin/env bash
set -euo pipefail
DEPLOY_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$DEPLOY_ROOT" -B "$DEPLOY_ROOT/build" -DCMAKE_BUILD_TYPE=Release "$@"
cmake --build "$DEPLOY_ROOT/build" --target g1_control hand_control -j "${BUILD_JOBS:-2}"
