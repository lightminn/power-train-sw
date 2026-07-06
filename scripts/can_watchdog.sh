#!/bin/bash
# can0 TX 웻지 자동복구 워치독 (Jetson 호스트에서 sudo 로 실행).
#
# 배경: mttcan 드라이버는 bus-off 가 반복되면(모터 PWM 노이즈로 CAN TX 에러 폭풍)
# TX 큐를 영구히 안 비우는 상태(웻지)에 빠진다 — berr 카운터 0·상태 ERROR-ACTIVE 로
# "멀쩡해 보이는데" qdisc 백로그에 프레임이 갇히고 모든 send 가 ENOBUFS. 재현·검증:
# 2026-07-07 (bus-off 796회 누적 후 TX 0/30 → down/up 만으로 30/30 부활).
#
# 감지: qdisc 백로그 >0 인데 Sent 카운터가 2초(2연속 샘플) 동안 정지 → 웻지 판정
#       (정상 부하는 백로그가 ms 단위로 빠지고 Sent 가 계속 증가하므로 오탐 없음).
# 복구: ip link down/up + txqueuelen 재설정 (~0.2s). 제어측은 프레임 몇 개 유실 후
#       재개 — teleop 서버는 CanError 흡수라 안 죽고, 코너 stale→FAULT 시 □ 재무장.
#
# 실행 (Jetson 호스트):
#   sudo nohup bash scripts/can_watchdog.sh > /tmp/can_watchdog.log 2>&1 &
# 중지: sudo pkill -f can_watchdog
IF="${1:-can0}"
INTERVAL=1

get_stats() {
    # "sent backlog_pkts" 출력 (파싱 실패 시 빈 문자열)
    tc -s qdisc show dev "$IF" 2>/dev/null | awk '
        /Sent/    { sent=$2 }
        /backlog/ { gsub(/p/,"",$3); pkts=$3 }
        END       { if (sent != "") print sent, pkts+0 }'
}

echo "[can_watchdog] 시작 — $IF 감시 (간격 ${INTERVAL}s)"
prev_sent=""
prev_backlog=0
while true; do
    read -r sent backlog <<< "$(get_stats)"
    if [[ -n "$sent" && "$backlog" -gt 0 && "$sent" == "$prev_sent" && "$prev_backlog" -gt 0 ]]; then
        echo "[can_watchdog] $(date '+%F %T') TX 웻지 감지 (backlog ${backlog}p, Sent 정지) → $IF 리셋"
        ip link set "$IF" down
        ip link set "$IF" up
        ip link set "$IF" txqueuelen 1000
        prev_sent=""
        prev_backlog=0
        sleep 1
        continue
    fi
    prev_sent="$sent"
    prev_backlog="$backlog"
    sleep "$INTERVAL"
done
