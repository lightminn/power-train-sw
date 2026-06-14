#!/usr/bin/env bash
# 노트북 측 수신기 ① — 저지연 네이티브 뷰어 (오버레이 없음).
#
# 용도: 원격주행(teleop) — 사람이 영상만 보고 조종. 파이썬/cv2 를 거치지 않고
#   gst 가 직접 디코드·표시해 지연이 가장 낮다. 좌표 박스 오버레이가 필요하면
#   ②번 recv_yolo3d.py 를 쓴다 (지연 더 큼).
#
# 사용: ./recv_stream.sh [PORT] [JETSON_HOST] [SRT_LATENCY_MS]
#       기본값      5000    jetson-orin.local  60
#   - 더 낮은 지연(깨끗한 링크): 송신측 --srt-latency 30 + 여기 3번째 인자 30
#   - 혼잡 WiFi 안정성 우선: 양쪽 100~120
#   ※ SRT latency 는 송·수신 중 큰 값으로 협상되므로 양쪽을 같이 맞춰야 한다.

PORT="${1:-5000}"
HOST="${2:-jetson-orin.local}"
LAT="${3:-60}"

# mDNS 이름(.local)은 IPv6 link-local(fe80::…)로 먼저 풀리는데 gst SRT URI 는
#   scope id 를 못 실어 접속이 무한 실패한다 (HIL 확인). A 레코드(IPv4)만 강제.
IPV4="$(getent ahostsv4 "$HOST" 2>/dev/null | awk 'NR==1{print $1}')"
[ -n "$IPV4" ] && HOST="$IPV4"

# SRT caller: 송신측(로봇)이 listener 라 노트북이 접속해 들어간다 — 수신기를
#   껐다 켜도 송신은 살아 있다. latency: ARQ 재전송 지연 예산(ms).
# avdec max-threads=1: 프레임 단위 멀티스레드 디코딩은 스레드 수만큼 프레임을
#   쌓아 "프레임 개수" 고정 지연을 만든다 — 저fps 에서 초 단위로 증폭. 단일스레드.
# sync=false: 클럭 동기 없이 도착 즉시 표시 (저지연). 창에서 'f' 로 전체화면.
exec gst-launch-1.0 -v \
    srtsrc uri="srt://${HOST}:${PORT}?mode=caller&latency=${LAT}" \
    ! tsdemux ! h264parse ! avdec_h264 max-threads=1 ! videoconvert \
    ! autovideosink sync=false
