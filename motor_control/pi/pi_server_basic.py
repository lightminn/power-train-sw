"""
⛔ DEPRECATED: 정본은 teleop_command/:9000 경로다.
이 스크립트는 구 Raspberry Pi 데모용이며 실모터 사용을 권장하지 않는다.

Pi에서 실행: python3 robot_server2.py
노트북 robot_client2.py에서 속도값 수신 → ODrive input_vel 직접 전달
"""
import socket
import time
import odrive
from odrive.enums import *

try:
    from pi.legacy_command import serve_command_connection
except ModuleNotFoundError:  # direct ``python pi_server_basic.py`` execution
    from legacy_command import serve_command_connection

COMMAND_PORT = 9000
AXIS_NUM     = 1
MAX_VEL      = 5.0

print("🤖 ODrive 검색 중...")
drv = odrive.find_any()
print(f"✅ ODrive 연결 성공! FW: {drv.fw_version_major}.{drv.fw_version_minor}.{drv.fw_version_revision}  vbus: {drv.vbus_voltage:.2f}V")

ax = drv.axis1 if AXIS_NUM == 1 else drv.axis0

print("⚙️ FULL_CALIBRATION_SEQUENCE 시작... (모터가 움직입니다)")
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
print(f"✅ ODrive 준비 완료")

# ── 소켓 서버 ──────────────────────────────────────
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', COMMAND_PORT))
server.listen(1)
print(f"🚀 명령 서버 대기 중 (포트 {COMMAND_PORT}) ...")

try:
    while True:
        conn, addr = server.accept()
        print(f"🎮 클라이언트 연결: {addr}")
        try:
            serve_command_connection(
                connection=conn,
                apply_command=lambda vel: setattr(ax.controller, "input_vel", vel),
                hold_command=lambda: setattr(ax.controller, "input_vel", 0.0),
                max_abs=MAX_VEL,
            )
        except OSError:
            pass
        finally:
            ax.controller.input_vel = 0.0
            conn.close()
            print("🔌 클라이언트 연결 해제 — 모터 정지")

except KeyboardInterrupt:
    print("\n🛑 서버 종료")
finally:
    ax.controller.input_vel = 0.0
    time.sleep(0.3)
    ax.requested_state = AxisState.IDLE
    server.close()
    print("✅ IDLE — 종료")
