#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${ATC_BUILD_DIR:-${project_dir}/build-release}"

cmake -S "${project_dir}" -B "${build_dir}" -G Ninja \
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}" \
  -DATC_BUILD_GUI=ON \
  -DATC_BUILD_TESTS=ON
cmake --build "${build_dir}" --parallel
ctest --test-dir "${build_dir}" --output-on-failure

printf 'Executável: %s\n' "${build_dir}/antenna-tilt-controller"
