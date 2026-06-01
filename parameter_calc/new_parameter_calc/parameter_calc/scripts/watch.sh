#!/bin/bash
# watch.sh — v4 본 실행 진행 + GPU 상태 통합 모니터.
#
# 사용:
#   bash watch.sh [interval_seconds]    # 기본 5초
#   bash watch.sh 2                     # 2초 간격
#
# 출력 한 줄:
#   [HH:MM:SS] step N/2000  f=X.XXXX  ETA Yh  |  GPU mem M/T MB  temp°C  pwr W  util%

INTERVAL="${1:-5}"

# ZETIN 로그 자동 탐지
find_log() {
    for f in $(ls -t /tmp/claude-*/-*Rocker*-Bogie*/*/tasks/*.output 2>/dev/null); do
        if grep -q "ZETIN GPU 가속 최적화" "$f" 2>/dev/null; then
            echo "$f"; return
        fi
    done
}

LOG=$(find_log)
if [ -z "$LOG" ]; then
    echo "⚠ ZETIN v4 로그 없음. GPU만 모니터링."
fi

# 시작 시각 (birth time)
if [ -n "$LOG" ]; then
    START_TS=$(stat -c %W "$LOG" 2>/dev/null)
    [ -z "$START_TS" ] || [ "$START_TS" = "0" ] && START_TS=$(stat -c %Y "$LOG")
fi

echo "========================================================================"
echo "  ZETIN v4 + GPU 통합 모니터  (${INTERVAL}초 간격, Ctrl+C 중단)"
echo "========================================================================"
if [ -n "$LOG" ]; then
    echo "v4 로그: $LOG"
    echo "시작:    $(date -d @$START_TS '+%Y-%m-%d %H:%M:%S')"
fi
echo "GPU 안전: <80°C  주의: 80-85°C  throttle 위험: 85°C+"
echo "------------------------------------------------------------------------"

# 헤더
printf "%-9s | %5s/2000 %-9s %4s  | %4s/4096MB %3s°C %4sW %3s%%  %s\n" \
       "시각" "step" "f_opt" "ETA" "mem" "tmp" "pwr" "u%" "상태"
echo "------------------------------------------------------------------------"

while true; do
    NOW=$(date +%H:%M:%S)

    # v4 진행
    STEP="-"
    FOPT="-"
    ETA="-"
    if [ -n "$LOG" ] && [ -f "$LOG" ]; then
        LAST=$(grep "differential_evolution step" "$LOG" 2>/dev/null | tail -1)
        if [ -n "$LAST" ]; then
            STEP=$(echo "$LAST" | awk '{print $3}' | tr -d ':')
            FOPT=$(echo "$LAST" | awk '{print $5}' | cut -c1-7)
            NOW_TS=$(date +%s)
            ELAPSED=$((NOW_TS - START_TS))
            if [ "$STEP" -gt 0 ] 2>/dev/null; then
                RATE=$((ELAPSED / STEP))  # sec/step
                REMAINING=$(( (2000 - STEP) * RATE ))
                if [ "$REMAINING" -ge 3600 ]; then
                    ETA="$(( REMAINING / 3600 ))h$(( (REMAINING % 3600) / 60 ))m"
                else
                    ETA="$(( REMAINING / 60 ))m"
                fi
            fi
        fi

        # 완료 체크
        if grep -q "최적화 완료" "$LOG" 2>/dev/null; then
            echo "------------------------------------------------------------------------"
            echo "[$NOW] 🏁 v4 최적화 완료!"
            grep -E "f_opt|소요 시간|저장" "$LOG" | tail -3
            break
        fi
    fi

    # GPU
    GPU_INFO=$(nvidia-smi --query-gpu=memory.used,temperature.gpu,power.draw,utilization.gpu \
                          --format=csv,noheader,nounits 2>/dev/null)
    IFS=',' read -ra V <<< "$GPU_INFO"
    MEM=$(echo ${V[0]} | xargs)
    TEMP=$(echo ${V[1]} | xargs)
    PWR=$(echo ${V[2]} | xargs)
    UTIL=$(echo ${V[3]} | xargs)

    # 상태 판정
    if [ -n "$TEMP" ] && [ "$TEMP" -ge 90 ]; then STATUS="🔥CRIT"
    elif [ -n "$TEMP" ] && [ "$TEMP" -ge 85 ]; then STATUS="⚠thrtl"
    elif [ -n "$TEMP" ] && [ "$TEMP" -ge 80 ]; then STATUS="⚠warn"
    else STATUS="✓ok"
    fi

    printf "%-9s | %5s     %-9s %4s  | %4s       %3s   %4s   %3s    %s\n" \
           "$NOW" "$STEP" "$FOPT" "$ETA" "$MEM" "$TEMP" "$PWR" "$UTIL" "$STATUS"

    sleep "$INTERVAL"
done
