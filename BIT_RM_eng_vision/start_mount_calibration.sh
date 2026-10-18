#!/usr/bin/env bash
# 独立安装角标定：camera + detect + pnp + comm + mount_calibration（不启动 tf）
# 参数见 src/camera_mount_calibration_pkg/config/mount_calibration.yaml

set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${ROOT}/log/camera_mount_calibration"
PIDS=()

source /opt/ros/humble/setup.bash
mkdir -p "${LOG_DIR}"

if systemctl is-active --quiet eng-real.service 2>/dev/null; then
  echo "[error] eng-real.service 正在运行；请先执行:" >&2
  echo "  sudo systemctl stop eng-real.service" >&2
  exit 1
fi

kill_calibration_pipeline() {
  pkill -f "ros2 run camera_pkg camera_node" 2>/dev/null || true
  pkill -f "ros2 run pose_detect_pkg detect_node" 2>/dev/null || true
  pkill -f "ros2 run pnp_solver_pkg pnp_node" 2>/dev/null || true
  pkill -f "ros2 run comm_pkg comm_node" 2>/dev/null || true
  pkill -f "ros2 run tf_pkg tf_node" 2>/dev/null || true
  pkill -f "ros2 run camera_mount_calibration_pkg mount_calibration_node" \
    2>/dev/null || true
  pkill -f "camera_pkg.camera_node" 2>/dev/null || true
  pkill -f "pose_detect_pkg.detect_node" 2>/dev/null || true
  pkill -f "pnp_solver_pkg.pnp_node" 2>/dev/null || true
  pkill -f "comm_pkg.comm_node" 2>/dev/null || true
  pkill -f "tf_pkg.tf_node" 2>/dev/null || true
  pkill -f "camera_mount_calibration_pkg.mount_calibration_node" \
    2>/dev/null || true
  sleep 0.3
}

cleanup() {
  echo
  echo "[exit] stopping mount calibration..."
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
  kill_calibration_pipeline
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[start] killing old manual pipeline processes..."
kill_calibration_pipeline

colcon build --packages-up-to \
  camera_pkg pose_detect_pkg pnp_solver_pkg comm_pkg \
  camera_mount_calibration_pkg
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"

echo "[start] camera_node"
ros2 run camera_pkg camera_node >"${LOG_DIR}/camera_node.log" 2>&1 &
PIDS+=($!)

echo "[start] detect_node"
ros2 run pose_detect_pkg detect_node >"${LOG_DIR}/detect_node.log" 2>&1 &
PIDS+=($!)

echo "[start] pnp_node"
ros2 run pnp_solver_pkg pnp_node >"${LOG_DIR}/pnp_node.log" 2>&1 &
PIDS+=($!)

echo "[start] comm_node scheduled after calibration subscriber (existing protocol only)"
(
  sleep 1.5
  exec ros2 run comm_pkg comm_node
) >"${LOG_DIR}/comm_node.log" 2>&1 &
PIDS+=($!)

echo "[start] mount_calibration_node (replaces tf_node during calibration)"
ros2 run camera_mount_calibration_pkg mount_calibration_node
