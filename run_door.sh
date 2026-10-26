#!/usr/bin/env bash
set -euo pipefail
DOOR_PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DOOR_PYTHON="$DOOR_PROJECT_DIR/.venv/bin/python"
if [[ ! -x "$DOOR_PYTHON" ]]; then
    echo "找不到项目解释器：$DOOR_PYTHON" >&2
    exit 1
fi
PYTHONPATH= exec "$DOOR_PYTHON" "$DOOR_PROJECT_DIR/door_demo.py" "$@"
