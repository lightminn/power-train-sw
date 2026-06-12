#!/usr/bin/env bash
# 노트북 측: Jetson 의 SRT H.264 스트림을 수신해 화면에 표시 (단순 뷰어).
# 좌표 오버레이까지 보려면 recv_yolo3d.py 사용.
# 사용: ./recv_stream.sh [PORT] [JETSON_HOST]   (default 5000 jetson-orin.local)

PORT="${1:-5000}"
HOST="${2:-jetson-orin.local}"

# SRT caller: 송신측(로봇)이 listener 라 노트북이 접속해 들어간다 — 수신기를
#   껐다 켜도 송신은 살아 있다. latency=120: ARQ 재전송 지연 예산(ms) —
#   순수 UDP RTP 와 달리 패킷 손실을 재전송으로 복구 (혼잡 WiFi 대비).
# avdec max-threads=1: 프레임 단위 멀티스레드 디코딩은 스레드 수(~코어 수)만큼
#   프레임을 쌓아야 해 "프레임 개수" 고정 지연 발생 — 송신 fps 가 낮을수록 초 단위
#   랙으로 증폭된다. 640x480 H.264 는 싱글스레드로 충분.
exec gst-launch-1.0 -v \
    srtsrc uri="srt://${HOST}:${PORT}?mode=caller&latency=120" \
    ! tsdemux ! h264parse ! avdec_h264 max-threads=1 ! videoconvert \
    ! autovideosink sync=false
