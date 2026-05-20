#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR=""
BUILD_DIR_SET=0
QUANT_CNOC=0
STRICT_ATTENTION=0
MODEL="toy"
COMPARE_MODE=""

usage() {
  cat <<USAGE
Usage: $0 [--quant-cnoc] [--strict-attention] [--model toy|synthetic-cnoc|medium-synthetic-cnoc]
          [--compare route|quant|strict] [--build-dir DIR]

  --quant-cnoc  Use a separate CNOC_QUANT_GOLDEN=ON build and compare
                type4/type5 traces with integer Q4.4 cNoC semantics.
  --strict-attention
                Include type5 Attention numeric fields in quantized comparison.
  --model       Select the workload model. Default: toy.
  --compare     Select comparison strictness. Default: route for non-quant
                builds, quant for --quant-cnoc, strict for --strict-attention.
  --build-dir   Override the CMake build directory.
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
        echo "[ERROR] --model requires toy, synthetic-cnoc, or medium-synthetic-cnoc." >&2
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

case "${MODEL}" in
  toy|synthetic-cnoc|medium-synthetic-cnoc) ;;
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
    BUILD_DIR="${ROOT_DIR}/build_cnoc_quant"
  else
    BUILD_DIR="${ROOT_DIR}/build"
  fi
elif [[ "${BUILD_DIR}" != /* ]]; then
  BUILD_DIR="${ROOT_DIR}/${BUILD_DIR}"
fi

TRACE_DIR="${BUILD_DIR}/trace_regression"
LOCK_DIR="${TRACE_DIR}.lock"
MODEL_DIR="${TRACE_DIR}/m"

mkdir -p "${TRACE_DIR}"

while ! mkdir "${LOCK_DIR}" 2>/dev/null; do
  echo "[INFO] Waiting for another trace regression to finish..."
  sleep 1
done
trap 'rmdir "${LOCK_DIR}"' EXIT

if [[ "${QUANT_CNOC}" == "1" ]]; then
  cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCNOC_QUANT_GOLDEN=ON -DENABLE_CNOC_MFU=ON
else
  cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCNOC_QUANT_GOLDEN=OFF -DENABLE_CNOC_MFU=OFF
fi
cmake --build "${BUILD_DIR}" -j"$(nproc)"

MODEL_ARGS=()
if [[ "${MODEL}" == "synthetic-cnoc" || "${MODEL}" == "medium-synthetic-cnoc" ]]; then
  mkdir -p "${MODEL_DIR}"
  MODEL_FILE="${MODEL_DIR}/mdl.txt"
  WEIGHT_FILE="${MODEL_DIR}/w.txt"
  INPUT_FILE="${MODEL_DIR}/in.txt"

  if [[ "${MODEL}" == "medium-synthetic-cnoc" ]]; then
    cat > "${MODEL_FILE}" <<'MODEL_EOF'
Input 16 1 1
Embedding 32 16
MatMul 16 64
MatMul 64 128
GeGLU 64
MatMul 64 64
Add 64 1
MatMul 64 16
MODEL_EOF

    : > "${WEIGHT_FILE}"
    for row in $(seq 0 31); do
      for col in $(seq 0 15); do
        awk "BEGIN { printf \"%.4f%s\", ((((${row}+${col}) % 7) - 3) / 16.0), (${col} == 15) ? \"\\n\" : \" \" }" \
          >> "${WEIGHT_FILE}"
      done
    done
    for row in $(seq 0 63); do
      for col in $(seq 0 15); do
        awk "BEGIN { printf \"%.4f \", ((((${row}*3+${col}) % 9) - 4) / 16.0) }" \
          >> "${WEIGHT_FILE}"
      done
      printf '0.0000\n' >> "${WEIGHT_FILE}"
    done
    for row in $(seq 0 127); do
      for col in $(seq 0 63); do
        awk "BEGIN { printf \"%.4f \", ((((${row}+${col}*5) % 11) - 5) / 32.0) }" \
          >> "${WEIGHT_FILE}"
      done
      printf '0.0000\n' >> "${WEIGHT_FILE}"
    done
    for row in $(seq 0 63); do
      for col in $(seq 0 63); do
        awk "BEGIN { printf \"%.4f \", (${row} == ${col}) ? 0.5000 : ((((${row}+${col}) % 5) - 2) / 64.0) }" \
          >> "${WEIGHT_FILE}"
      done
      printf '0.0000\n' >> "${WEIGHT_FILE}"
    done
    for row in $(seq 0 15); do
      for col in $(seq 0 63); do
        awk "BEGIN { printf \"%.4f \", ((((${row}*7+${col}) % 13) - 6) / 64.0) }" \
          >> "${WEIGHT_FILE}"
      done
      printf '0.0000\n' >> "${WEIGHT_FILE}"
    done

    cat > "${INPUT_FILE}" <<'INPUT_EOF'
1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
INPUT_EOF
  else
  cat > "${MODEL_FILE}" <<'MODEL_EOF'
Input 4 1 1
Embedding 8 4
MatMul 4 8
Attention 8 4 2 2
MatMul 4 8
GeGLU 4
MatMul 4 4
Add 4 1
MatMul 4 2
MODEL_EOF

  : > "${WEIGHT_FILE}"
  # Embedding 8x4.  Input tokens below use ids 1..4, but all rows are present
  # so the model satisfies the normal NoCDAS weight loader contract.
  for row in 0 1 2 3 4 5 6 7; do
    printf '%s %s %s %s\n' \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+1) / 16.0 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+2) / 16.0 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+3) / 16.0 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+4) / 16.0 }")" \
      >> "${WEIGHT_FILE}"
  done
  # MatMul 4->8 used as Q/K/V projection for Attention.
  for row in 0 1 2 3 4 5 6 7; do
    printf '%s %s %s %s 0.0000\n' \
      "$(awk "BEGIN { printf \"%.4f\", (${row}%4 == 0) ? 0.7500 : 0.1250 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}%4 == 1) ? 0.7500 : -0.1250 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}%4 == 2) ? 0.7500 : 0.2500 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}%4 == 3) ? 0.7500 : -0.2500 }")" \
      >> "${WEIGHT_FILE}"
  done
  # MatMul 4->8 feeding GeGLU.
  for row in 0 1 2 3 4 5 6 7; do
    printf '%s %s %s %s 0.0000\n' \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+1) / 32.0 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+2) / -32.0 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+3) / 32.0 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row}+4) / -32.0 }")" \
      >> "${WEIGHT_FILE}"
  done
  # MatMul 4->4 after GeGLU.
  for row in 0 1 2 3; do
    printf '%s %s %s %s 0.0000\n' \
      "$(awk "BEGIN { printf \"%.4f\", (${row} == 0) ? 1.0000 : 0.0000 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row} == 1) ? 1.0000 : 0.0000 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row} == 2) ? 1.0000 : 0.0000 }")" \
      "$(awk "BEGIN { printf \"%.4f\", (${row} == 3) ? 1.0000 : 0.0000 }")" \
      >> "${WEIGHT_FILE}"
  done
  # Final MatMul 4->2.
  cat >> "${WEIGHT_FILE}" <<'WEIGHT_EOF'
1.0000 0.0000 0.5000 0.0000 0.0000
0.0000 1.0000 0.0000 0.5000 0.0000
WEIGHT_EOF

  cat > "${INPUT_FILE}" <<'INPUT_EOF'
1 2 3 4
INPUT_EOF
  fi

  MODEL_ARGS=(-NNmodel "${MODEL_FILE}" -NNweight "${WEIGHT_FILE}" -NNinput "${INPUT_FILE}")
fi

(
  cd "${ROOT_DIR}"
  "${BUILD_DIR}/NoCDASim" "${MODEL_ARGS[@]}" -trace
  if [[ "${QUANT_CNOC}" == "1" ]]; then
    cp trace.log "${TRACE_DIR}/trace_${MODEL}_cpp_quant_golden.log"
  else
    cp trace.log "${TRACE_DIR}/trace_${MODEL}_cpp_golden.log"
  fi

  "${BUILD_DIR}/NoCDASim" "${MODEL_ARGS[@]}" -rtl_router -trace
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
