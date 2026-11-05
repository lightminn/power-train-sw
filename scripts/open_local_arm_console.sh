#!/usr/bin/env bash
# Open the PR #12 console against the already running local bench runtime.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
token=/tmp/arm-gui-session/ops_console.token
if [[ ! -r "$token" ]]; then
    echo "로컬 팔 시험 런타임의 인증 토큰이 없습니다: $token" >&2
    exit 1
fi
exec /usr/bin/python3 -m operator_console.app \
    --host 127.0.0.1 --ops-host 127.0.0.1 --ops-port 19001 \
    --ops-token-file "$token" --arm-telemetry-port 15707 \
    --metadata-port 15703 --telemetry-port 15704 \
    --chassis-telemetry-port 15705 --environment-telemetry-port 15708
