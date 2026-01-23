#!/bin/bash

# TensorRT engine build script for YOLOv8 ONNX.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MODELS_DIR="$PROJECT_DIR/models"

ONNX_PATH="$MODELS_DIR/yolov8n.onnx"
ENGINE_PATH="$MODELS_DIR/yolov8n_fp16.engine"
INPUT_NAME="images"
INPUT_SIZE=640
WORKSPACE=4096
FP16=1
DYNAMIC_SHAPES=0
DRY_RUN=0
TRTEXEC_BIN=""

usage() {
    cat <<'EOF'
Usage: build_trt_engine.sh [options]

Options:
  --onnx <path>        ONNX model path (default: models/yolov8n.onnx)
  --engine <path>      Output engine path (default: models/yolov8n_fp16.engine)
  --input-size <int>   Input size (default: 640)
  --workspace <mb>     TensorRT workspace size in MB (default: 4096)
  --trtexec <path>     Override trtexec path
  --fp16               Enable FP16 (default)
  --no-fp16            Disable FP16 (FP32)
  --dynamic-shapes     Add min/opt/max shapes (for dynamic ONNX)
  --dry-run            Print command only
  -h, --help           Show this help
EOF
}

log() {
    echo "[build_trt_engine] $*"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --onnx)
            ONNX_PATH="$2"
            shift 2
            ;;
        --engine)
            ENGINE_PATH="$2"
            shift 2
            ;;
        --input-size)
            INPUT_SIZE="$2"
            shift 2
            ;;
        --workspace)
            WORKSPACE="$2"
            shift 2
            ;;
        --trtexec)
            TRTEXEC_BIN="$2"
            shift 2
            ;;
        --fp16)
            FP16=1
            shift
            ;;
        --no-fp16)
            FP16=0
            shift
            ;;
        --dynamic-shapes)
            DYNAMIC_SHAPES=1
            shift
            ;;
        --dry-run)
            DRY_RUN=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ -z "$TRTEXEC_BIN" ]]; then
    if command -v trtexec >/dev/null 2>&1; then
        TRTEXEC_BIN="$(command -v trtexec)"
    elif [[ -x "/usr/src/tensorrt/bin/trtexec" ]]; then
        TRTEXEC_BIN="/usr/src/tensorrt/bin/trtexec"
    elif [[ -x "/usr/local/tensorrt/bin/trtexec" ]]; then
        TRTEXEC_BIN="/usr/local/tensorrt/bin/trtexec"
    else
        echo "Error: trtexec not found. Install TensorRT or pass --trtexec."
        exit 1
    fi
fi

if [[ ! -f "$ONNX_PATH" ]]; then
    echo "Error: ONNX model not found: $ONNX_PATH"
    exit 1
fi

ENGINE_DIR="$(dirname "$ENGINE_PATH")"
mkdir -p "$ENGINE_DIR"

if [[ -f "$ENGINE_PATH" ]]; then
    ts="$(date +%Y%m%d_%H%M%S)"
    backup="${ENGINE_PATH}.bak.${ts}"
    log "Existing engine found, moving to $backup"
    mv "$ENGINE_PATH" "$backup"
fi

TRTEXEC_HELP="$("$TRTEXEC_BIN" --help 2>/dev/null || true)"

CMD=(
    "$TRTEXEC_BIN"
    "--onnx=$ONNX_PATH"
    "--saveEngine=$ENGINE_PATH"
)

if [[ "$DYNAMIC_SHAPES" -eq 1 ]]; then
    CMD+=(
        "--minShapes=${INPUT_NAME}:1x3x${INPUT_SIZE}x${INPUT_SIZE}"
        "--optShapes=${INPUT_NAME}:1x3x${INPUT_SIZE}x${INPUT_SIZE}"
        "--maxShapes=${INPUT_NAME}:1x3x${INPUT_SIZE}x${INPUT_SIZE}"
    )
fi

if echo "$TRTEXEC_HELP" | grep -q -- "--explicitBatch"; then
    CMD+=("--explicitBatch")
fi

WORKSPACE_ARG=""
if echo "$TRTEXEC_HELP" | grep -q -- "--memPoolSize"; then
    WORKSPACE_ARG="--memPoolSize=workspace:${WORKSPACE}"
elif echo "$TRTEXEC_HELP" | grep -q -- "--workspace"; then
    WORKSPACE_ARG="--workspace=${WORKSPACE}"
fi

if [[ -n "$WORKSPACE_ARG" ]]; then
    CMD+=("$WORKSPACE_ARG")
fi

if [[ "$FP16" -eq 1 ]]; then
    CMD+=("--fp16")
fi

log "Using trtexec: $TRTEXEC_BIN"
log "ONNX: $ONNX_PATH"
log "Engine: $ENGINE_PATH"
log "Input: ${INPUT_NAME} 1x3x${INPUT_SIZE}x${INPUT_SIZE}"

if [[ "$DRY_RUN" -eq 1 ]]; then
    printf '%q ' "${CMD[@]}"
    echo
    exit 0
fi

"${CMD[@]}"

if [[ ! -s "$ENGINE_PATH" ]]; then
    echo "Error: engine build failed, output not created: $ENGINE_PATH"
    exit 1
fi

log "Engine created: $ENGINE_PATH"
