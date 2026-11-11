#!/usr/bin/env bash
# Build + test inside the workspace-local pixi env. Low-RAM safe (1 worker, -j2).
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh
pixi run colcon build --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
pixi run colcon test --event-handlers console_direct+
pixi run colcon test-result --all --verbose
