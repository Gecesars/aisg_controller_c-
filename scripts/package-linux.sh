#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${ATC_BUILD_DIR:-${project_dir}/build-release}"
dist_dir="${project_dir}/dist"

bash "${project_dir}/scripts/build-linux.sh"
mkdir -p "${dist_dir}"
cpack --config "${build_dir}/CPackConfig.cmake" -B "${dist_dir}"

printf 'Pacote criado em: %s\n' "${dist_dir}"
