#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export OMNI_KIT_ACCEPT_EULA=Y
export OMNI_KIT_ALLOW_ROOT=1
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-8}"
export PYTHONUNBUFFERED=1
unset HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export PYTHONPATH="$repo_dir${PYTHONPATH:+:$PYTHONPATH}"
cd "$repo_dir"
exec "${ROBOAPI_PYTHON:-$repo_dir/.venv/bin/python}" -m policies.roboapi.run \
  --headless --device "${ROBOAPI_DEVICE:-cpu}" "$@"
