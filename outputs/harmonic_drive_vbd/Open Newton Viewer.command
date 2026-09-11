#!/bin/sh
set -eu
cd "$(dirname "$0")"
export PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
SIM_PYTHON="${SIM_PYTHON:-.venv/bin/python}"
if [ ! -x "$SIM_PYTHON" ]; then
  printf '%s\n' 'Create .venv and install requirements-viewer.txt as described in README.md.'
  exit 1
fi
exec "$SIM_PYTHON" native_viewer.py "$@"
