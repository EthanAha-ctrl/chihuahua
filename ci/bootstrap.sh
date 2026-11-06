#!/usr/bin/env bash
set -Eeuo pipefail
trap 'printf "FATAL: bootstrap failed at line %s\n" "$LINENO" >&2' ERR
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
command -v python3 >/dev/null || { echo 'FATAL: Python 3.11+ is required to bootstrap.' >&2; exit 1; }
exec python3 -B ci/bootstrap.py "$@"
