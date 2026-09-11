#!/bin/sh
set -eu
cd "$(dirname "$0")"
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
fi
.venv/bin/python -m pip install -r requirements.txt
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
.venv/bin/python simulate.py --duration 7.1
.venv/bin/python build_viewer.py
open replay_rom.html
