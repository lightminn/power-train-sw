"""
Pi에서 실행: python3 robot_pi.py
  - GStreamer: 카메라 영상 TCP 스트리밍 (포트 5000)
  - ODrive:    속도 명령 수신 및 모터 제어 (포트 9000)
"""
import os
import socket
import subprocess
import time
import odrive
from odrive.enums import *

COMMAND_PORT = 9000
VIDEO_PORT   = 5000
AXIS_NUM     = 1
MAX_VEL      = 5.0

CAMERA_DEV = '/dev/video0'
GST_CMD = (
    f"gst-launch-1.0 v4l2src device={CAMERA_DEV} do-timestamp=true "
    "! image/jpeg,width=1280,height=720,framerate=30/1 "
    f"! tcpserversink host=0.0.0.0 port={VIDEO_PORT}"
)

# ── ODrive 초기화 ─────────────────────────────────
print("🤖 ODrive 검색 중...")
drv = odrive.find_any()
print(f"✅ ODrive 연결  FW:{drv.fw_version_major}.{drv.fw_version_minor}.{drv.fw_version_revision}  vbus:{drv.vbus_voltage:.2f}V")

ax = drv.axis1 if AXIS_NUM == 1 else drv.axis0

print("⚙️  FULL_CALIBRATION_SEQUENCE 시작...")
ax.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
while ax.current_state != AxisState.IDLE:
    time.sleep(0.1)

if ax.error != 0:
    print(f"❌ 캘리 실패  axis:{ax.error:#x}  motor:{ax.motor.error:#x}  enc:{ax.encoder.error:#x}")
    exit()

print("🔄 VELOCITY_CONTROL + PASSTHROUGH 설정")
ax.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
ax.controller.config.input_mode   = InputMode.PASSTHROUGH

print("🔒 CLOSED_LOOP_CONTROL 진입")
ax.requested_state = AxisState.CLOSED_LOOP_CONTROL
time.sleep(0.5)

if ax.current_state != 8:
    print(f"❌ 폐루프 진입 실패  state:{ax.current_state}  error:{ax.error:#x}")
    exit()

ax.controller.input_vel = 0.0
print("✅ ODrive 준비 완료")

# ── GStreamer 영상 서버 시작 (ODrive 준비 후) ─────
print(f"📷 카메라 장치 대기 중 ({CAMERA_DEV})...")
for _ in range(20):
    if os.path.exists(CAMERA_DEV):
        break
    print(f"   {CAMERA_DEV} 아직 없음, 대기...")
    time.sleep(1)
else:
    print(f"❌ {CAMERA_DEV} 사용 불가")
    exit()

time.sleep(1)  # USB 안정화 대기
print(f"📷 영상 스트리밍 시작 (포트 {VIDEO_PORT})...")
gst_proc = subprocess.Popen(GST_CMD, shell=True)
print(f"   PID: {gst_proc.pid}")

# ── 명령 소켓 서버 ────────────────────────────────
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', COMMAND_PORT))
server.listen(1)
print(f"🚀 명령 서버 대기 중 (포트 {COMMAND_PORT}) ...")

try:
    while True:
        conn, addr = server.accept()
        print(f"🎮 클라이언트 연결: {addr}")
        buf = b''
        try:
            while True:
                data = conn.recv(64)
                if not data:
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    try:
                        vel = float(line.decode().strip())
                        vel = max(-MAX_VEL, min(MAX_VEL, vel))
                        ax.controller.input_vel = vel
                    except ValueError:
                        pass
        except OSError:
            pass
        finally:
            ax.controller.input_vel = 0.0
            conn.close()
            print("🔌 클라이언트 연결 해제 — 모터 정지")

except KeyboardInterrupt:
    print("\n🛑 종료 중...")
finally:
    ax.controller.input_vel = 0.0
    time.sleep(0.3)
    ax.requested_state = AxisState.IDLE
    server.close()
    gst_proc.terminate()
    gst_proc.wait()
    print("✅ IDLE — 종료")
