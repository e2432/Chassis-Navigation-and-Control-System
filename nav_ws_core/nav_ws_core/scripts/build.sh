#!/usr/bin/env bash
set -e -o pipefail
CORE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$CORE_ROOT/scripts/env.sh"
cd "$CORE_ROOT"
colcon build --symlink-install "$@"
