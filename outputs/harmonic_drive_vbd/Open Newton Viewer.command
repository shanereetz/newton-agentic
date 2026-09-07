#!/bin/sh
set -eu
cd "$(dirname "$0")"
export WARP_CACHE_PATH="/Users/virtualaxis/Documents/Codex/2026-09-07/cre/work/newton_warp_cache"
export PYTHONUNBUFFERED=1
SIM_PYTHON="/Users/virtualaxis/Documents/Codex/2026-09-07/cre/work/newton-env/bin/python"
if [ ! -x "$SIM_PYTHON" ]; then
  printf '%s\n' 'The prepared Python environment is missing. See README.md for setup.'
  read -r reply
  exit 1
fi
if ! "$SIM_PYTHON" native_viewer.py; then
  printf '\n%s\n' 'Newton Viewer stopped with an error. Copy the error above for troubleshooting.'
  read -r reply
  exit 1
fi
