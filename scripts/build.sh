#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_DIR}"

source ./zephyr-env.sh

cmake -B build -DBOARD=robomaster_c -DBOARD_ROOT=.
cmake --build build
