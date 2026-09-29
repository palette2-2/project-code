#!/usr/bin/env bash
# Compatibility alias for the root CMake build.
set -euo pipefail
exec "$(dirname -- "${BASH_SOURCE[0]}")/build.sh" "$@"
