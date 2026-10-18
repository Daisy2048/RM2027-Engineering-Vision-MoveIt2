#!/usr/bin/env bash
# 一键启动：读取 config/pipeline.yaml 决定 real/virtual 及可选节点
# 各节点参数见 src/<包>/config/*.yaml（节点启动时自行读取）
# Ctrl+C 退出并杀掉全部子进程

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash

# 解析 pipeline.yaml 为 PIPELINE_* 变量
eval "$(python3 - "${ROOT}/config/pipeline.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}

def emit(key, value):
    print(f"PIPELINE_{key.upper()}={value}")

source = str(config.get("camera_source", "")).strip().lower()
if source not in ("real", "virtual"):
    raise SystemExit(f"[error] camera_source 必须是 real 或 virtual: {source!r}")
emit("camera_source", source)
for key in ("build", "foxglove", "recorder", "comm"):
    emit(key, 1 if config.get(key, False) else 0)
PY
)"

LOG_DIR="${ROOT}/log/runtime_${PIPELINE_CAMERA_SOURCE}/$(date +%Y%m%d_%H%M%S)"
PIDS=()

if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT}/install/setup.bash"
fi
mkdir -p "${LOG_DIR}"
ln -sfn "${LOG_DIR}" "${ROOT}/log/runtime_${PIPELINE_CAMERA_SOURCE}/latest"

kill_pipeline() {
  pkill -f "ros2 run camera_pkg camera_node" 2>/dev/null || true
  pkill -f "ros2 run virtual_camera_pkg virtual_camera_node" 2>/dev/null || true
  pkill -f "ros2 run pose_detect_pkg detect_node" 2>/dev/null || true
  pkill -f "ros2 run pnp_solver_pkg pnp_node" 2>/dev/null || true
  pkill -f "ros2 run tf_pkg tf_node" 2>/dev/null || true
  pkill -f "ros2 run comm_pkg comm_node" 2>/dev/null || true
  pkill -f "ros2 run recorder_pkg recorder_node" 2>/dev/null || true
  pkill -f "camera_pkg.camera_node" 2>/dev/null || true
  pkill -f "virtual_camera_pkg.virtual_camera_node" 2>/dev/null || true
  pkill -f "pose_detect_pkg.detect_node" 2>/dev/null || true
  pkill -f "pnp_solver_pkg.pnp_node" 2>/dev/null || true
  pkill -f "tf_pkg.tf_node" 2>/dev/null || true
  pkill -f "comm_pkg.comm_node" 2>/dev/null || true
  pkill -f "recorder_pkg.recorder_node" 2>/dev/null || true
  if [[ "${PIPELINE_FOXGLOVE}" == "1" ]]; then
    pkill -f "ros2 run foxglove_bridge foxglove_bridge" 2>/dev/null || true
    pkill -f "foxglove_bridge" 2>/dev/null || true
  fi
  sleep 0.3
}

cleanup() {
  echo
  echo "[exit] killing pipeline..."
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  kill_pipeline
  wait 2>/dev/null || true
  echo "[exit] done"
}

trap cleanup EXIT INT TERM

echo "[start] killing old processes..."
kill_pipeline

if [[ "${PIPELINE_BUILD}" == "1" ]]; then
  colcon build
fi
if [[ ! -f "${ROOT}/install/setup.bash" ]]; then
  echo "[error] install/setup.bash 不存在；请先 colcon build" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"

if [[ "${PIPELINE_CAMERA_SOURCE}" == "real" ]]; then
  echo "[start] camera_node"
  ros2 run camera_pkg camera_node >"${LOG_DIR}/camera_node.log" 2>&1 &
  PIDS+=($!)
else
  echo "[start] virtual_camera_node"
  ros2 run virtual_camera_pkg virtual_camera_node >"${LOG_DIR}/virtual_camera_node.log" 2>&1 &
  PIDS+=($!)
fi

echo "[start] detect_node"
ros2 run pose_detect_pkg detect_node >"${LOG_DIR}/detect_node.log" 2>&1 &
PIDS+=($!)

echo "[start] pnp_node"
ros2 run pnp_solver_pkg pnp_node >"${LOG_DIR}/pnp_node.log" 2>&1 &
PIDS+=($!)

echo "[start] tf_node"
ros2 run tf_pkg tf_node >"${LOG_DIR}/tf_node.log" 2>&1 &
PIDS+=($!)

if [[ "${PIPELINE_COMM}" == "1" ]]; then
  echo "[start] comm_node"
  ros2 run comm_pkg comm_node >"${LOG_DIR}/comm_node.log" 2>&1 &
  PIDS+=($!)
fi

# recorder 作为旁路录制，崩溃不拖累核心链路，不加入 PIDS
if [[ "${PIPELINE_RECORDER}" == "1" ]]; then
  echo "[start] recorder_node (non-critical, isolated)"
  ros2 run recorder_pkg recorder_node >"${LOG_DIR}/recorder_node.log" 2>&1 &
fi

if [[ "${PIPELINE_FOXGLOVE}" == "1" ]]; then
  echo "[start] foxglove_bridge"
  ros2 run foxglove_bridge foxglove_bridge >"${LOG_DIR}/foxglove_bridge.log" 2>&1 &
  PIDS+=($!)
fi

echo
echo "${PIPELINE_CAMERA_SOURCE} pipeline running (Ctrl+C to stop)"
[[ "${PIPELINE_FOXGLOVE}" == "1" ]] && \
  echo "  foxglove_bridge WebSocket: ws://localhost:8765"
echo "logs: ${LOG_DIR}/"

# 任一核心子进程退出都视为整条链路故障，退出后 cleanup 终止其余节点
set +e
wait -n "${PIDS[@]}"
CHILD_STATUS=$?
set -e
if [[ "${CHILD_STATUS}" -eq 0 ]]; then
  CHILD_STATUS=1
fi
echo "[error] pipeline child exited (status=${CHILD_STATUS}); stopping all nodes" >&2
exit "${CHILD_STATUS}"
