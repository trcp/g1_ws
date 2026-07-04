#!/usr/bin/env bash
set -euo pipefail

MODEL_NAME="yoloe-26x-seg.pt"
MODEL_URL="https://github.com/ultralytics/assets/releases/download/v8.4.0/${MODEL_NAME}"
TEXT_ENCODER_NAME="mobileclip2_b.ts"
CACHE_DIR="/root/.cache/ultralytics/weights"
CACHE_FILE="${CACHE_DIR}/${MODEL_NAME}"
APP_FILE="/app/${MODEL_NAME}"
TEXT_ENCODER_CACHE_FILE="${CACHE_DIR}/${TEXT_ENCODER_NAME}"
TEXT_ENCODER_APP_FILE="/app/${TEXT_ENCODER_NAME}"

mkdir -p "${CACHE_DIR}"

if [ -d "${APP_FILE}" ] || [ -L "${APP_FILE}" ]; then
  rm -rf "${APP_FILE}"
fi

if [ ! -s "${CACHE_FILE}" ]; then
  echo "[weights] Downloading ${MODEL_NAME}"
  echo "[weights] Source: ${MODEL_URL}"
  if ! curl --fail --location --show-error \
    --connect-timeout 30 \
    --max-time 1800 \
    --retry 5 \
    --retry-delay 5 \
    --retry-all-errors \
    --continue-at - \
    --output "${CACHE_FILE}" \
    "${MODEL_URL}"; then
    rm -f "${CACHE_FILE}"
    echo "[weights] ERROR: ${MODEL_NAME} is not cached and could not be downloaded." >&2
    echo "[weights] Connect to Wi-Fi once, or copy it to ${CACHE_FILE}, then start again." >&2
    exit 1
  fi
fi

ln -sf "${CACHE_FILE}" "${APP_FILE}"
echo "[weights] Ready: ${APP_FILE} -> ${CACHE_FILE}"

if [ -s "${TEXT_ENCODER_CACHE_FILE}" ]; then
  ln -sf "${TEXT_ENCODER_CACHE_FILE}" "${TEXT_ENCODER_APP_FILE}"
  echo "[weights] Ready: ${TEXT_ENCODER_APP_FILE} -> ${TEXT_ENCODER_CACHE_FILE}"
fi
