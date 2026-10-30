#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
exec "${ROBOAPI_TEST_PYTHON:-python}" -m pytest --noconftest tests/test_roboapi_*.py "$@"
