"""
AK40-10 CAN 제어 라이브러리 (Jetson + socketcan) 수정본
- 위치 제어는 기어비 곱셈 없이 출력축 각도(out_deg) 직결!
"""

import can
import struct
import time
import math

# ============================================================
# 데이터시트 상수 (AK40-10)
# ============================================================
GEAR_RATIO = 10.0        # 10:1 감속
POLE_PAIRS = 14

RATED_ERPM = int(370 * POLE_PAIRS)   # 5180
NOLOAD_ERPM = int(435 * POLE_PAIRS)    # 6090

# CAN packet IDs
PKT_SET_DUTY = 0
PKT_SET_BRAKE = 2
PKT_SET_RPM = 3
PKT_SET_POS_SPD = 6
PKT_STATUS_1 = 41

DEFAULT_MAX_CUR_A = 5.0
DEFAULT_SPD_ERPM = 1500
DEFAULT_ACC_ERPM_S2 = 6000

# ============================================================
# 짐벌 운동학
# ============================================================
OFFSET_X_MM = 0.0
OFFSET_Y_MM = -100.0
OFFSET_Z_MM = 50.0


def calc_pan_tilt_deg(target_x_mm, target_y_mm, target_z_mm,
                      ox=OFFSET_X_MM, oy=OFFSET_Y_MM, oz=OFFSET_Z_MM):
    xm = target_x_mm - ox
    ym = target_y_mm - oy
    zm = target_z_mm - oz
    pan_deg = math.degrees(math.atan2(xm, zm))
    tilt_deg = math.degrees(math.atan2(ym, math.hypot(xm, zm)))
    return pan_deg, tilt_deg

# ============================================================
# AK40-10 드라이버
# ============================================================


class AK40:
    def __init__(self, bus, motor_id, name=""):
        self.bus = bus
        self.mid = motor_id
        self.name = name or f"id{motor_id}"
        # 💡 [수정] 모터 내부 각도가 아닌 바깥 출력축 각도(out_deg)로 통일
        self.pos_out_deg = 0.0
        self.spd_erpm = 0
        self.cur_a = 0.0
        self.temp_c = 0
        self.fault = 0

    def _safe_send(self, msg):
        try:
            self.bus.send(msg, timeout=0.01)
            return True
        except can.CanOperationError:
            return False

    def _send(self, packet_id, data):
        ext_id = (packet_id << 8) | self.mid
        msg = can.Message(arbitration_id=ext_id,
                          data=data, is_extended_id=True)
        return self._safe_send(msg)

    def set_origin_here(self):
        # 5번 ID가 ORIGIN 셋
        return self._send(5, bytes([0x01]))

    def send_rpm_out(self, out_rpm):
        """RPM 제어는 ERPM이 필요하므로 기어비와 극쌍수를 곱합니다."""
        erpm = int(-out_rpm * GEAR_RATIO * POLE_PAIRS)
        erpm = max(-NOLOAD_ERPM, min(NOLOAD_ERPM, erpm))
        return self._send(PKT_SET_RPM, struct.pack(">i", erpm))

    def send_brake(self, current_a):
        """전류 기반 브레이크 (VESC: mA). 0~20A 클램프 (20A=hw 절대 최대)."""
        cur = max(0.0, min(20.0, float(current_a)))
        return self._send(PKT_SET_BRAKE, struct.pack(">i", int(cur * 1000)))

    def send_duty(self, duty):
        """직접 듀티 (-0.95~0.95 클램프, VESC: ×100000)."""
        d = max(-0.95, min(0.95, float(duty)))
        return self._send(PKT_SET_DUTY, struct.pack(">i", int(d * 100000)))

    def send_pos_out(self, out_deg, spd_erpm=DEFAULT_SPD_ERPM, acc_erpm_s2=DEFAULT_ACC_ERPM_S2):
        """💡 [핵심 수정] 위치 제어는 기어비 곱셈 없이 날것 그대로 쏩니다."""
        spd = max(-32768, min(32767, int(spd_erpm)))
        acc = max(-32768, min(32767, int(acc_erpm_s2)))
        data = struct.pack(">ihh", int(out_deg * 10000), spd, acc)
        return self._send(PKT_SET_POS_SPD, data)

    def _parse_status(self, data):
        pos, spd, cur, temp, fault = struct.unpack(">hhhbb", data[:8])
        # 💡 [수정] status에서 오는 pos 값도 이미 출력축 기준입니다.
        self.pos_out_deg = pos / 10.0
        self.spd_erpm = spd * 10
        self.cur_a = cur / 100.0
        self.temp_c = temp
        self.fault = fault

    def poll(self, timeout=0.05):
        latest = None
        t_end = time.time() + timeout
        while True:
            remain = t_end - time.time()
            if remain <= 0 and latest is not None:
                break
            wait = max(remain, 0.0) if latest is None else 0.0
            msg = self.bus.recv(timeout=wait)
            if msg is None:
                break
            pkt = (msg.arbitration_id >> 8) & 0xFF
            nid = msg.arbitration_id & 0xFF
            if pkt == PKT_STATUS_1 and nid == self.mid and len(msg.data) >= 8:
                latest = msg.data
        if latest is None:
            return False
        self._parse_status(latest)
        return True

    def move_rel_out(self, delta_out_deg, hold_sec=None,
                     spd_erpm=DEFAULT_SPD_ERPM, acc_erpm_s2=DEFAULT_ACC_ERPM_S2,
                     hz=20, tol_deg=2.0, max_cur_a=DEFAULT_MAX_CUR_A,
                     timeout_extra=2.0, verbose=True):

        if not self.poll(timeout=0.3):
            raise RuntimeError(f"[{self.name}] status 미수신")

        # 💡 [핵심 수정] 타겟 각도 계산 시 GEAR_RATIO 뻥튀기 삭제!
        target = self.pos_out_deg + delta_out_deg

        if hold_sec is None:
            cruise_dps = (spd_erpm / POLE_PAIRS) * 6.0
            ramp_t = spd_erpm / max(acc_erpm_s2, 1)
            travel_t = abs(delta_out_deg * GEAR_RATIO) / max(cruise_dps, 1)
            hold_sec = travel_t + 2*ramp_t + timeout_extra

        if verbose:
            print(
                f"  [{self.name}] start={self.pos_out_deg:+.1f}° → target={target:+.1f}°")

        end = time.time() + hold_sec
        period = 1.0/hz
        last_send = 0.0
        aborted = False

        while time.time() < end:
            now = time.time()
            if now - last_send >= period:
                self.send_pos_out(target, spd_erpm, acc_erpm_s2)
                last_send = now
            self.poll(timeout=0.005)

            if abs(self.cur_a) > max_cur_a:
                self.send_rpm_out(0)
                aborted = True
                break
            if self.fault != 0:
                self.send_rpm_out(0)
                aborted = True
                break
            if abs(self.pos_out_deg - target) < tol_deg and abs(self.spd_erpm) < 200:
                break

        self.poll(timeout=0.1)
        err = self.pos_out_deg - target
        if verbose:
            print(
                f"  [{self.name}] done={self.pos_out_deg:+.1f}° err={err:+.2f}° cur={self.cur_a:+.2f}A")
        return not aborted

    def move_rel_out_safe(self, delta_out_deg,
                          spd_erpm=DEFAULT_SPD_ERPM, acc_erpm_s2=DEFAULT_ACC_ERPM_S2,
                          hz=20, tol_deg=2.0, max_chunk_out_deg=300.0,
                          max_cur_a=DEFAULT_MAX_CUR_A, verbose=True):

        # 💡 [핵심 수정] 남은 각도 계산 시 뻥튀기 삭제
        remaining = abs(delta_out_deg)
        sign = 1 if delta_out_deg >= 0 else -1

        while remaining > 0.5:
            self.send_rpm_out(0)
            time.sleep(0.05)
            self.set_origin_here()
            time.sleep(0.15)
            self.poll(timeout=0.3)

            chunk_out = min(remaining, max_chunk_out_deg) * sign
            if verbose:
                print(f"  [{self.name}] chunk out={chunk_out:+.1f}°")

            self.move_rel_out(chunk_out, spd_erpm=spd_erpm,
                              acc_erpm_s2=acc_erpm_s2, hz=hz,
                              tol_deg=tol_deg, max_cur_a=max_cur_a,
                              verbose=verbose)
            remaining -= abs(chunk_out)

    def stop(self, n=5):
        for _ in range(n):
            self.send_rpm_out(0)  # ID 3은 절대 죽지 않는 안전한 패킷!
            time.sleep(0.05)

# ============================================================
# 컨텍스트 매니저
# ============================================================


class CANSession:
    def __init__(self, channel='can0', interface='socketcan'):
        self.channel = channel
        self.interface = interface
        self.motors = []

    def __enter__(self):
        self.bus = can.interface.Bus(
            channel=self.channel, interface=self.interface)
        return self

    def add_motor(self, motor_id, name=""):
        m = AK40(self.bus, motor_id, name)
        self.motors.append(m)
        return m

    def __exit__(self, exc_type, exc_val, exc_tb):
        for m in self.motors:
            try:
                m.stop()
            except:
                pass
        try:
            self.bus.shutdown()
        except:
            pass
        return False

# ============================================================
# 데모
# ============================================================


def demo_full_functions():
    with CANSession('can0') as sess:
        m = sess.add_motor(motor_id=10, name="pan")

        print("\n=== 🛠️ 1. 초기화 및 영점 설정 (set_origin_here) ===")
        m.send_rpm_out(0)
        time.sleep(0.1)
        m.set_origin_here()
        time.sleep(0.2)
        m.poll(timeout=0.5)
        print(f"  [상태] 위치={m.pos_out_deg:.1f}°, 온도={m.temp_c}℃, 에러={m.fault}")
        time.sleep(1)

        print("\n=== 🔄 2. 상대 위치 제어 (move_rel_out) ===")
        print("  [+90° 이동]")
        m.move_rel_out(90, spd_erpm=1500, acc_erpm_s2=6000)
        time.sleep(0.5)
        print("  [-90° 이동 (원위치)]")
        m.move_rel_out(-90, spd_erpm=1500, acc_erpm_s2=6000)
        time.sleep(1)

        print("\n=== 🚀 3. 분할 안전 위치 제어 (move_rel_out_safe) ===")
        print("  [+360° 회전 (내부적으로 청크 분할 이동)]")
        m.move_rel_out_safe(+360, spd_erpm=2000, acc_erpm_s2=6000)
        time.sleep(1)

        print("\n=== 🏎️ 4. 연속 RPM 제어 (send_rpm_out) ===")
        print("  [출력축 30 RPM으로 3초간 회전]")
        # RPM 제어는 워치독 방지를 위해 루프 안에서 지속 송신
        end_time = time.time() + 3.0
        while time.time() < end_time:
            m.send_rpm_out(30.0)
            time.sleep(0.05)

        print("\n=== 🛑 5. 브레이크 제어 (send_brake) ===")
        print("  [전류 2.0A로 강제 브레이크 체결 (1초 유지)]")
        # 돌고 있던 모터에 브레이크를 걸어 급정거 후 홀딩
        end_time = time.time() + 1.0
        while time.time() < end_time:
            m.send_brake(2.0)
            time.sleep(0.05)

        print("\n=== 💤 6. 안전 정지 (stop) ===")
        print("  [모터 전력 차단 및 대기 상태 진입]")
        m.stop()
        print("\n✨ 모든 데모 시연이 안전하게 완료되었습니다!")


def demo_core_features():
    with CANSession('can0') as sess:
        m = sess.add_motor(motor_id=10, name="pan")

        print("\n=== 🛠️ [준비] 영점 설정 ===")
        # 1. 0 RPM을 쏴서 모터와 통신이 되는지(모닝콜) 확인
        for _ in range(3):
            m.send_rpm_out(0)
            time.sleep(0.05)

        m.set_origin_here()
        time.sleep(0.2)

        if not m.poll(timeout=1.0):
            print("🚨 [에러] 모터 응답 없음! ./can_setup.sh 를 실행하세요.")
            return
        print(f"  영점이 설정되었습니다. (현재 온도: {m.temp_c}℃)\n")

        # --------------------------------------------------
        # 1. 위치 제어
        # --------------------------------------------------
        print("=== 🎯 1. 위치 제어 (Position Control) ===")
        print("  [설명] 지정한 각도(90도)로 부드럽고 정확하게 이동합니다.")
        m.move_rel_out(90, spd_erpm=1500, acc_erpm_s2=6000)
        time.sleep(1)

        # --------------------------------------------------
        # 2. 속도 제어
        # --------------------------------------------------
        print("\n=== 🏎️ 2. 속도 제어 (Velocity Control) ===")
        print("  [설명] 출력축 기준 40 RPM으로 3초 동안 연속 회전합니다.")
        end_time = time.time() + 3.0
        while time.time() < end_time:
            m.send_rpm_out(40.0)
            time.sleep(0.05)

        # --------------------------------------------------
        # 3. 포지션 홀딩 (진짜 위치 잠금!)
        # --------------------------------------------------
        print("\n=== 🛑 3. 포지션 홀딩 (Position Lock) ===")
        print("  [설명] 현재 도착한 위치를 강제로 꽉 붙잡습니다.")
        print("  👉 (지금 모터 축을 손으로 돌려보세요! 절대 안 돌아갈 겁니다.)")

        end_time = time.time() + 10.0
        # 💡 [핵심] 현재 위치를 타겟으로 고정
        hold_target = m.pos_out_deg

        while time.time() < end_time:
            # 💡 [핵심] 가장 강력한 힘(1500 ERPM 제한)으로 현재 위치를 사수하라!
            m.send_pos_out(hold_target, spd_erpm=1500, acc_erpm_s2=6000)
            time.sleep(0.05)
        print("  홀딩 해제!\n")

        print("=== 💤 안전 정지 및 데모 종료 ===")
        m.stop()


if __name__ == "__main__":
    demo_core_features()
