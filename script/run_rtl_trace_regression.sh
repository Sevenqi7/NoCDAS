#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR=""
BUILD_DIR_SET=0
QUANT_CNOC=0
STRICT_ATTENTION=0
MODEL="toy"
COMPARE_MODE=""
MATMUL_CTX_SLOTS=1

usage() {
  cat <<USAGE
Usage: $0 [--quant-cnoc] [--strict-attention] [--model toy]
          [--compare route|quant|strict] [--build-dir DIR] [--matmul-ctx-slots N]

  --quant-cnoc  Use a separate CNOC_QUANT_GOLDEN=ON build and compare
                type4/type5 traces with integer Q4.4 cNoC semantics.
  --strict-attention
                Include type5 Attention numeric fields in quantized comparison.
  --model       Select the workload model. Only toy is supported. Default: toy.
  --compare     Select comparison strictness. Default: route for non-quant
                builds, quant for --quant-cnoc, strict for --strict-attention.
  --build-dir   Override the CMake build directory.
  --matmul-ctx-slots  Compile Router with the requested MatMul/Linear context slots (1-4).
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --quant-cnoc)
      QUANT_CNOC=1
      shift
      ;;
    --strict-attention)
      STRICT_ATTENTION=1
      QUANT_CNOC=1
      shift
      ;;
    --model)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --model requires toy." >&2
        usage
        exit 2
      fi
      MODEL="$2"
      shift 2
      ;;
    --compare)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --compare requires route, quant, or strict." >&2
        usage
        exit 2
      fi
      COMPARE_MODE="$2"
      shift 2
      ;;
    --build-dir)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --build-dir requires a directory argument." >&2
        usage
        exit 2
      fi
      BUILD_DIR="$2"
      BUILD_DIR_SET=1
      shift 2
      ;;
    --matmul-ctx-slots)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --matmul-ctx-slots requires an integer argument." >&2
        usage
        exit 2
      fi
      MATMUL_CTX_SLOTS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

case "${MATMUL_CTX_SLOTS}" in
  ''|*[!0-9]*)
    echo "[ERROR] --matmul-ctx-slots must be a positive integer." >&2
    usage
    exit 2
    ;;
esac
if [[ "${MATMUL_CTX_SLOTS}" -lt 1 || "${MATMUL_CTX_SLOTS}" -gt 4 ]]; then
  echo "[ERROR] --matmul-ctx-slots must be in the range [1, 4]." >&2
  usage
  exit 2
fi

case "${MODEL}" in
  toy) ;;
  *)
    echo "[ERROR] Unsupported model: ${MODEL}" >&2
    usage
    exit 2
    ;;
esac

if [[ -z "${COMPARE_MODE}" ]]; then
  if [[ "${STRICT_ATTENTION}" == "1" ]]; then
    COMPARE_MODE="strict"
  elif [[ "${QUANT_CNOC}" == "1" ]]; then
    COMPARE_MODE="quant"
  else
    COMPARE_MODE="route"
  fi
fi

case "${COMPARE_MODE}" in
  route)
    QUANT_CNOC=0
    STRICT_ATTENTION=0
    ;;
  quant)
    QUANT_CNOC=1
    ;;
  strict)
    QUANT_CNOC=1
    STRICT_ATTENTION=1
    ;;
  *)
    echo "[ERROR] Unsupported compare mode: ${COMPARE_MODE}" >&2
    usage
    exit 2
    ;;
esac

if [[ "${BUILD_DIR_SET}" == "0" ]]; then
  if [[ "${QUANT_CNOC}" == "1" ]]; then
    if [[ "${MATMUL_CTX_SLOTS}" == "1" ]]; then
      BUILD_DIR="${ROOT_DIR}/build_cnoc_quant"
    else
      BUILD_DIR="${ROOT_DIR}/build_cnoc_quant_matmul_s${MATMUL_CTX_SLOTS}"
    fi
  else
    if [[ "${MATMUL_CTX_SLOTS}" == "1" ]]; then
      BUILD_DIR="${ROOT_DIR}/build"
    else
      BUILD_DIR="${ROOT_DIR}/build_matmul_s${MATMUL_CTX_SLOTS}"
    fi
  fi
elif [[ "${BUILD_DIR}" != /* ]]; then
  BUILD_DIR="${ROOT_DIR}/${BUILD_DIR}"
fi

TRACE_DIR="${BUILD_DIR}/trace_regression"
LOCK_FILE="${ROOT_DIR}/.rtl_trace_regression.lockfile"
LEGACY_LOCK_DIR="${ROOT_DIR}/.rtl_trace_regression.lock"

mkdir -p "${TRACE_DIR}"

if [[ -d "${LEGACY_LOCK_DIR}" ]]; then
  rmdir "${LEGACY_LOCK_DIR}" 2>/dev/null || true
fi

exec 9>"${LOCK_FILE}"
while ! flock -n 9; do
  echo "[INFO] Waiting for another trace regression to finish writing shared trace.log..."
  sleep 1
done
trap 'flock -u 9' EXIT

if [[ "${QUANT_CNOC}" == "1" ]]; then
  cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCNOC_QUANT_GOLDEN=ON -DENABLE_CNOC_MFU=ON -DMATMUL_CTX_SLOTS="${MATMUL_CTX_SLOTS}"
else
  cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCNOC_QUANT_GOLDEN=OFF -DENABLE_CNOC_MFU=OFF -DMATMUL_CTX_SLOTS="${MATMUL_CTX_SLOTS}"
fi
cmake --build "${BUILD_DIR}" -j"$(nproc)"

(
  cd "${ROOT_DIR}"
  rm -f trace.log
  "${BUILD_DIR}/NoCDASim" -trace
  if [[ "${QUANT_CNOC}" == "1" ]]; then
    cp trace.log "${TRACE_DIR}/trace_${MODEL}_cpp_quant_golden.log"
  else
    cp trace.log "${TRACE_DIR}/trace_${MODEL}_cpp_golden.log"
  fi

  rm -f trace.log
  "${BUILD_DIR}/NoCDASim" -rtl_router -trace
  if [[ "${QUANT_CNOC}" == "1" ]]; then
    cp trace.log "${TRACE_DIR}/trace_${MODEL}_rtl_quant_candidate.log"
  else
    cp trace.log "${TRACE_DIR}/trace_${MODEL}_rtl_candidate.log"
  fi
)

COMPARE_LOG="${TRACE_DIR}/compare_${MODEL}_${COMPARE_MODE}.log"
if [[ "${COMPARE_MODE}" == "route" ]]; then
  python3 "${ROOT_DIR}/script/compare_traces.py" \
    --ignore-cnoc-values \
    "${TRACE_DIR}/trace_${MODEL}_cpp_golden.log" \
    "${TRACE_DIR}/trace_${MODEL}_rtl_candidate.log" | tee "${COMPARE_LOG}"

  echo "[PASS] RTL trace regression matched routing/type smoke baseline with cNoC values ignored."
else
  COMPARE_ARGS=(--quant-cnoc)
  if [[ "${COMPARE_MODE}" == "strict" ]]; then
    COMPARE_ARGS+=(--strict-attention)
  fi
  python3 "${ROOT_DIR}/script/compare_traces.py" \
    "${COMPARE_ARGS[@]}" \
    "${TRACE_DIR}/trace_${MODEL}_cpp_quant_golden.log" \
    "${TRACE_DIR}/trace_${MODEL}_rtl_quant_candidate.log" | tee "${COMPARE_LOG}"

  if [[ "${COMPARE_MODE}" == "strict" ]]; then
    echo "[PASS] RTL-owned quantized cNoC trace matched routing/type and Attention-inclusive integer values."
  else
    echo "[PASS] RTL-owned quantized cNoC trace matched routing/type and non-Attention integer values."
  fi
fi

if rg -n "rtl_cnoc_owned|dpi_cnoc_compute|legacy_cnoc_dpi" \
  "${ROOT_DIR}/src" "${ROOT_DIR}/rtl" "${ROOT_DIR}/script" \
  --glob '!run_rtl_trace_regression.sh' >/dev/null; then
  echo "[ERROR] Legacy cNoC ownership path marker found after regression." >&2
  rg -n "rtl_cnoc_owned|dpi_cnoc_compute|legacy_cnoc_dpi" \
    "${ROOT_DIR}/src" "${ROOT_DIR}/rtl" "${ROOT_DIR}/script" \
    --glob '!run_rtl_trace_regression.sh' >&2
  exit 1
fi
