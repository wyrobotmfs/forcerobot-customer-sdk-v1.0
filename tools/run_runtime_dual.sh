#!/usr/bin/env bash
# 在包根目录执行： sudo ./tools/run_runtime_dual.sh ./config <网口名>
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export LD_LIBRARY_PATH="${ROOT}/tools:${ROOT}/vendor/lib:${LD_LIBRARY_PATH:-}"
exec "${ROOT}/tools/runtime_dual" "$@"
