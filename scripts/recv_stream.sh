#!/usr/bin/env bash
# 노트북 측: Jetson 에서 보낸 UDP RTP H.264 영상을 수신해 화면에 표시.
# 사용: ./recv_stream.sh [PORT]   (default 5000)

PORT="${1:-5000}"

# rtpjitterbuffer: 무선 간헐 지연/순서뒤바뀜을 150ms 버퍼로 흡수 (고정 지연 +150ms)
exec gst-launch-1.0 -v \
    udpsrc port="$PORT" \
        caps='application/x-rtp,encoding-name=H264,payload=96' \
    ! rtpjitterbuffer latency=150 \
    ! rtph264depay ! avdec_h264 ! videoconvert \
    ! autovideosink sync=false
