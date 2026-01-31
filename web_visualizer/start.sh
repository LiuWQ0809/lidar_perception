#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONFIG_PATH="${CONFIG_PATH:-${SCRIPT_DIR}/../config/fusion_config.yaml}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"
IMAGE_TOPIC="${IMAGE_TOPIC:-}"
MARKER_TOPIC="${MARKER_TOPIC:-}"
JPEG_QUALITY="${JPEG_QUALITY:-80}"
MAX_FPS="${MAX_FPS:-15}"

ARGS=("--config" "${CONFIG_PATH}" "--host" "${HOST}" "--port" "${PORT}" "--jpeg-quality" "${JPEG_QUALITY}" "--max-fps" "${MAX_FPS}")

if [[ -n "${IMAGE_TOPIC}" ]]; then
  ARGS+=("--image-topic" "${IMAGE_TOPIC}")
fi

if [[ -n "${MARKER_TOPIC}" ]]; then
  ARGS+=("--marker-topic" "${MARKER_TOPIC}")
fi

python3 "${SCRIPT_DIR}/web_visualizer_node.py" "${ARGS[@]}"
