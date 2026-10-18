#!/usr/bin/env bash
# 一键运行各包单元测试（unittest + pytest）
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash

if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT}/install/setup.bash"
else
  echo "[error] install/setup.bash 不存在，请先 colcon build" >&2
  exit 1
fi

cd "${ROOT}"
FAILED=0

for test_file in src/*/test/test_*.py; do
  echo "[test] ${test_file}"
  if ! python3 "${test_file}"; then
    FAILED=1
  fi
done

if [[ "${FAILED}" -ne 0 ]]; then
  echo "[error] 部分测试失败" >&2
  exit 1
fi

echo "[ok] 全部单元测试通过"
