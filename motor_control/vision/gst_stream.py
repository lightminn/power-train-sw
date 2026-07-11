#!/usr/bin/env python3
"""Jetson → 노트북 영상 송신 GStreamer 파이프라인 빌더 (vision 스크립트 공용).

인코더 (소프트웨어 전용):
    Jetson **Orin Nano 에는 NVENC 하드웨어 인코더가 아예 없다** (Orin NX/AGX 만
    탑재). `nvv4l2h264enc` 는 NVENC 를 쓰는 플러그인이라 Orin Nano 에선 어떤
    드라이버/마운트 조합으로도 동작하지 않으며, GPU 인코딩 대안도 없다.
    따라서 SW 인코딩이 유일한 경로 — 기본은 x264 (zerolatency 튜닝, 동급
    비트레이트에서 openh264 보다 화질·지연 우위). x264enc 는
    gstreamer1.0-plugins-ugly 필요 — 구이미지엔 없으므로 openh264 폴백 유지.

전송 (SRT):
    순수 UDP RTP 는 패킷 손실 복구가 없어 (rtpjitterbuffer 는 순서·지터만 흡수)
    대회장 혼잡 WiFi 에서 블록노이즈·프리징이 난다. SRT 는 UDP 기반 저지연에
    ARQ 재전송 복구를 더한 프로토콜 — latency 파라미터(ms)가 재전송 지연 예산.
    송신측이 listener: 로봇이 스트림을 서빙하고 노트북(caller)이 접속하므로
    수신측이 죽었다 살아나도 송신 파이프라인이 끊기지 않는다 (재접속 허용).
    wait-for-connection=false: 수신자가 없어도 송신 루프가 블록되지 않음.

cv2.VideoWriter 의 GStreamer 백엔드는 dustynv 컨테이너의 opencv-python(pip)
빌드에 미포함 → subprocess + gst-launch (fdsrc fd=0 에 raw BGR 프레임 write).
"""

ENCODERS = ("x264", "openh264")
# Orin Nano benchmark selection: nvvidconv is used only with x264 when its
# measured benefit clears the acceptance rule; otherwise keep CPU conversion.
X264_CONVERSION = "videoconvert"


def build_conversion_tokens(encoder: str, x264_conversion: str) -> list[str]:
    """Return raw-BGR conversion tokens without coupling the openh264 fallback."""
    if encoder == "openh264" or x264_conversion == "videoconvert":
        return ["videoconvert", "!", "video/x-raw,format=I420"]
    if x264_conversion == "nvvidconv":
        return [
            "videoconvert", "!", "video/x-raw,format=BGRx",
            "!", "nvvidconv", "!", "video/x-raw,format=I420",
        ]
    raise ValueError(
        "x264_conversion must be videoconvert or nvvidconv: "
        f"{x264_conversion!r}"
    )


def build_gst_command(port: int, width: int, height: int, fps: int,
                      encoder: str = "x264", bitrate_kbps: int = 3000,
                      latency_ms: int = 60) -> list:
    """gst-launch argv — stdin 의 raw BGR 프레임을 H.264/MPEG-TS/SRT 로 송신.

    수신 (노트북): scripts/recv_stream.sh <port> <jetson-host>
                  또는 scripts/recv_yolo3d.py (좌표 오버레이 합성).
    """
    if encoder == "x264":
        # zerolatency: B-frame 0·슬라이스 스레딩, superfast: 6코어 A78 가 YOLO 와
        # 공존 가능한 프리셋. key-int-max=30: 키프레임 주기 — 손실 후 화질 복구 한도.
        enc = ["x264enc", "tune=zerolatency", "speed-preset=superfast", "threads=2",
               f"bitrate={bitrate_kbps}", "key-int-max=30"]
    elif encoder == "openh264":
        # 폴백 (구이미지에 plugins-ugly 없음). complexity=low: 인코더가 CPU 독식해
        # 검출 루프 굶기는 것 방지. scene-change-detection=false: IDR burst 억제.
        enc = ["openh264enc", f"bitrate={bitrate_kbps * 1000}", "gop-size=30",
               "complexity=low", "scene-change-detection=false"]
    else:
        raise ValueError(f"encoder must be one of {ENCODERS}: {encoder!r}")
    conversion = build_conversion_tokens(encoder, X264_CONVERSION)

    return [
        "gst-launch-1.0",
        "fdsrc", "fd=0", "do-timestamp=true",
        "!", "rawvideoparse",
             "format=bgr",
             f"width={width}", f"height={height}",
             f"framerate={fps}/1",
        "!", *conversion,
        "!", *enc,
        # config-interval=-1: 모든 키프레임에 SPS/PPS — 수신자가 중간 합류해도 디코딩 가능
        "!", "h264parse", "config-interval=-1",
        # alignment=7: TS 패킷 188B×7 단위 출력 (SRT 페이로드에 맞는 저지연 청크)
        "!", "mpegtsmux", "alignment=7",
        "!", "srtsink", f"uri=srt://:{port}?mode=listener&latency={latency_ms}",
             "wait-for-connection=false", "sync=false", "async=false",
    ]
