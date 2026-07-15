"""모터 HIL 사전 점검 — 전원 올리기 전에 돌린다.

    # 젯슨 컨테이너 안 (powertrain_jetson 또는 powertrain_ros)
    python3 /workspace/scripts/preflight_hil.py                 # 6륜
    python3 /workspace/scripts/preflight_hil.py --four-wheel    # 🛠️ 4륜 (중륜 보드=부하모터)

**모터를 돌리지 않는다.** 읽기만 한다. 통과해야 HIL 을 시작한다.

────────────────────────────────────────────────────────────────────────
왜 필요한가 — 과거에 여기서 다 터졌다
────────────────────────────────────────────────────────────────────────
· **좀비 teleop** 이 계속 `v=0` 을 명령해 새 테스트와 싸웠다.
· **캘리브레이션이 RAM-only** 라 전원 사이클마다 사라진다 → arm 은 되는데 **모터가 안 돈다.**
· `chassis_node` 와 `teleop_server` 를 **동시에** 띄워 같은 모터에 상반된 명령이 갔다.
· 텔레메트리만 보고 통과시켰다가 **바퀴가 실제로는 안 돌고 있었다**(HALL 코깅존).
"""
import os
import subprocess
import sys

sys.path.insert(0, os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"))

OK, WARN, FAIL = "✅", "⚠️ ", "❌"
_results = []

# 🛠️ 4륜 모드 — 중간 ODrive 보드(node 13/14 = 중륜)를 부하모터(다이나모)에 쓰고 있을 때.
#    이 보드가 버스에 없으면 CornerModule 이 stale 을 보고 corner FAULT → **전체 E-stop** 이 뜬다.
#    그래서 중륜을 아예 기하·매핑에서 빼고 조향 4륜만 돌린다.
FOUR_WHEEL = "--four-wheel" in sys.argv


def _geometry():
    from chassis.kinematics import default_geometry, four_wheel_geometry
    return four_wheel_geometry() if FOUR_WHEEL else default_geometry()


def _wheel_map():
    from chassis.chassis_manager import DEFAULT_WHEEL_MAP, FOUR_WHEEL_MAP
    return FOUR_WHEEL_MAP if FOUR_WHEEL else DEFAULT_WHEEL_MAP


def check(name, fn):
    try:
        level, detail = fn()
    except Exception as exc:                       # 점검 자체가 죽어도 계속 간다
        level, detail = FAIL, f"점검 실패: {type(exc).__name__}: {exc}"
    _results.append((level, name, detail))
    print(f"{level} {name}\n    {detail}", flush=True)


# ── 1. CAN 버스 ──────────────────────────────────────────────────────────

def can_up():
    """⚠️ `ip` 는 powertrain_ros 컨테이너에 없다 → sysfs 로 확인한다."""
    try:
        out = subprocess.run(["ip", "-details", "link", "show", "can0"],
                             capture_output=True, text=True).stdout
    except FileNotFoundError:
        out = ""

    if out:
        if "state UP" not in out:
            return FAIL, "can0 이 DOWN → 호스트에서 `bash scripts/can_setup.sh`"
        if "loopback on" in out:
            return FAIL, ("can0 이 LOOPBACK 모드다(버스 무음) → "
                          "`ip link set can0 type can loopback off`")
        bitrate = next((w for w in out.split()
                        if w.isdigit() and len(w) >= 6), "?")
        return OK, f"can0 UP (bitrate {bitrate}). **500000 이어야 한다.**"

    # sysfs 폴백
    path = "/sys/class/net/can0"
    if not os.path.isdir(path):
        return FAIL, "can0 이 없다 → 호스트에서 `bash scripts/can_setup.sh`"
    try:
        with open(os.path.join(path, "operstate")) as f:
            state = f.read().strip()
    except OSError:
        state = "?"
    if state not in ("up", "unknown"):             # CAN 은 보통 unknown 으로 뜬다
        return FAIL, f"can0 operstate={state} → 호스트에서 `bash scripts/can_setup.sh`"
    return WARN, (f"can0 존재 (operstate={state}). ⚠️ 이 컨테이너엔 `ip` 가 없어 "
                  "**bitrate·loopback 을 확인 못 한다** — 호스트에서 확인:\n"
                  "      ip -details link show can0   (500000 · loopback off 여야 한다)")


# ── 2. 좀비 프로세스 ─────────────────────────────────────────────────────

def _proc_state(pid):
    """/proc/PID/stat 의 상태 문자 (R/S/D/Z/T...). 못 읽으면 None."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(") ", 1)[1].split()[0]
    except (OSError, IndexError):
        return None


def no_zombies():
    out = subprocess.run(["pgrep", "-fa", "teleop|chassis|motor_gui"],
                         capture_output=True, text=True).stdout.strip()
    mine = str(os.getpid())

    alive, defunct = [], []
    for line in out.splitlines():
        if "preflight" in line or line.startswith(mine):
            continue
        pid = line.split(None, 1)[0]
        # ★ Z(defunct) 는 **이미 죽었고 CAN 소켓·모터를 아무것도 안 잡고 있다.**
        #   죽일 수도 없다(부모가 거두어야 사라진다). 위험이 아니므로 실패로 보지 않는다.
        (defunct if _proc_state(pid) == "Z" else alive).append(line)

    if alive:
        return FAIL, ("모터를 잡고 있을 수 있는 프로세스가 **살아있다** — 죽이고 다시 하라:\n      "
                      + "\n      ".join(alive[:4]))
    if defunct:
        return WARN, (
            f"defunct(Z) 프로세스 {len(defunct)}개 — **위험하지 않다**(이미 죽었고 CAN 소켓을 "
            "안 잡는다). 컨테이너 PID 1(Gateway)이 자식을 거두지 않아 쌓인다.\n"
            "      근본 해결: compose 에 `init: true`(tini) 추가. 지금은 무시해도 된다.")
    return OK, "teleop / chassis / motor_gui 프로세스 없음"


# ── 3. CAN 단독 소유권 락 ────────────────────────────────────────────────

_CAN_OWNER_SNAPSHOT = None
_CAN_OWNER_ERROR = None


def can_lock_owned():
    if _CAN_OWNER_SNAPSHOT is None:
        return FAIL, _CAN_OWNER_ERROR or "can0 owner lock 획득 실패"
    return OK, (
        "can0 owner lock 획득 "
        f"(pid={_CAN_OWNER_SNAPSHOT.pid}, "
        f"process={_CAN_OWNER_SNAPSHOT.process_name}, "
        f"path={_CAN_OWNER_SNAPSHOT.lock_path})"
    )


# ── 3.5 ★ CAN 버스에 ODrive 가 실제로 살아있는가 (수동 청취) ─────────────

def odrive_nodes_alive():
    """ODrive 는 CANSimple **하트비트(cmd 0x01)를 100 ms 마다 스스로 쏜다** → 아무것도
    안 보내고 2초 듣기만 해도 살아있는 노드를 알 수 있다.

    ★ 이게 4륜 모드의 핵심 점검이다. 기대하는 노드가 버스에 없으면 `CornerModule.tick()`
    이 stale 을 보고 corner FAULT → `ChassisManager` 가 **전체 E-stop** 을 건다.
    (AK 조향은 물어봐야 답하는 방식이라 수동 청취로는 안 보인다 — 여기선 구동만 본다.)
    """
    import can
    expect = {m.drive_node_id for m in _wheel_map()}

    try:
        bus = can.interface.Bus(channel="can0", interface="socketcan")
    except Exception as exc:
        return FAIL, f"can0 을 열 수 없다: {exc}"

    seen = {}
    deadline = __import__("time").time() + 2.0
    try:
        while __import__("time").time() < deadline:
            msg = bus.recv(timeout=0.2)
            if msg is None or msg.is_extended_id:
                continue
            node, cmd = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F
            if cmd == 0x01 and 1 <= node <= 63:            # 하트비트
                seen[node] = seen.get(node, 0) + 1
    finally:
        bus.shutdown()

    alive = {n for n in seen if seen[n] >= 2}              # 1회는 우연일 수 있다
    missing = sorted(expect - alive)
    extra = sorted(alive - expect)

    mode = "4륜" if FOUR_WHEEL else "6륜"
    desc = (f"{mode} 모드 기대 노드 {sorted(expect)} · "
            f"버스에서 관측된 ODrive {sorted(alive)}")

    if missing:
        hint = "      · 48 V 전원이 올라가 있는지, CAN 배선이 물려 있는지 본다."
        if not FOUR_WHEEL and set(missing) <= {13, 14}:
            # 중륜 보드만 없다 = 부하모터(다이나모)에 쓰는 중일 가능성이 높다.
            hint = ("      · 중륜 보드(13/14)만 없다 → 부하모터에 쓰는 중이면 "
                    "**--four-wheel 로 돌릴 것**.")
        elif not alive:
            hint = "      · **하나도 안 보인다 → 48 V 전원이 꺼져 있을 가능성이 가장 크다.**"
        return FAIL, (
            f"{desc}\n      **기대 노드 {missing} 이(가) 응답 없다** → 그대로 띄우면 "
            f"corner FAULT → **전체 E-stop**.\n" + hint)
    if extra:
        return WARN, (f"{desc}\n      기대에 없는 노드 {extra} 도 살아있다 — "
                      "부하모터(다이나모) 보드라면 정상이다. **명령은 안 간다.**")
    return OK, desc


# ── 4. 기하 — 운용 영역에 위험한 명령이 없는가 ───────────────────────────

def geometry_envelope():
    import math
    from chassis.kinematics import solve
    g = _geometry()
    worst_steer = worst_rev = 0.0
    for i in range(-15, 16):
        for j in range(-12, 13):
            r = solve(g, i / 10.0, j / 10.0)
            for wc in r.wheels.values():
                if not (math.isfinite(wc.steer_deg) and math.isfinite(wc.drive_mps)):
                    return FAIL, f"NaN/Inf 명령 발생 (v={i/10}, ω={j/10})"
                if abs(wc.steer_deg) > g.steer_limit_deg + 1e-6:
                    return FAIL, (f"조향 한계 초과 {wc.steer_deg:.1f}° > "
                                  f"{g.steer_limit_deg}° (v={i/10}, ω={j/10})")
                worst_steer = max(worst_steer, abs(wc.steer_deg))
                worst_rev = max(worst_rev, abs(wc.drive_turns_per_s))
    wb = (max(w.x for w in g.wheels) - min(w.x for w in g.wheels)) * 1000
    return OK, (f"축거 {wb:.0f} mm · 전 영역 위반 0건 "
                f"(최대 조향 {worst_steer:.1f}°, 최대 바퀴 {worst_rev:.2f} rev/s)")


# ── 5. ⚠️ 기하가 마지막 HIL 때와 다르다 ──────────────────────────────────

def geometry_changed_warning():
    from chassis.kinematics import solve
    g = _geometry()
    front = solve(g, 0.4, 0.4).wheels["front_left"].steer_deg
    rear = solve(g, 0.4, 0.4).wheels["rear_left"].steer_deg
    return WARN, (
        "**기하가 마지막 HIL(2026-07-05) 때와 다르다.** 설계팀 CAD 실측치로 교체됐다.\n"
        f"    같은 명령(v=0.4, ω=0.4)에서 앞 조향 +22.1° → **{front:+.1f}°**, "
        f"뒤 −22.1° → **{rear:+.1f}°**.\n"
        "    CAD 가 실물이므로 지금 값이 맞다. 다만 **첫 주행은 저속으로, 조향각을 눈으로 "
        "확인**하며 시작할 것.\n"
        "    (앞뒤 윤거가 705 / 585 mm 로 달라 조향각이 더 이상 거울상이 아니다 — "
        "설계팀 확인 대기)")


# ── 6. ODrive 캘리브레이션 (RAM-only) ────────────────────────────────────

def calibration_reminder():
    return WARN, (
        "**ODrive 캘리브레이션은 RAM-only** — 전원을 껐다 켰으면 반드시 다시 한다:\n"
        "      python3 drive/bl70200/can_calibrate_all.py     (6축, 약 6분)\n"
        "    안 하면 **arm 은 되는데 모터가 전혀 안 돈다**(폐루프 진입 거부).")


# ── 7. 코깅존 플로어 ─────────────────────────────────────────────────────

def cogging_floor():
    import math
    g = _geometry()
    circ = 2 * math.pi * g.wheel_radius_m
    v_min = 1.0 * circ
    return WARN, (
        f"**min_rev=1.0 이면 최저 속도가 {v_min:.2f} m/s** 다(상한 {g.drive_limit_mps}). "
        f"속도 범위 {g.drive_limit_mps/v_min:.2f}:1 — 사실상 '전속 아니면 정지'.\n"
        "    감속 힌트·정밀 접근·미션 정차가 전부 영향받는다.\n"
        "    실측 '깨끗한 대역'은 0.5~10 rev/s → **min_rev 0.6 검토** "
        "(docs/specs/2026-07-13-min-rev-speed-range.md).\n"
        "    ⚠️ 바꾸려면 **바퀴 띄우고 6바퀴가 실제로 도는지 육안 확인** 후에.")


# ── 8. 테스트 통과 여부 ──────────────────────────────────────────────────

def unit_tests():
    r = subprocess.run(
        [sys.executable, "-m", "pytest", "chassis/tests/", "corner_module/tests/",
         "-q", "--no-header", "-x"],
        cwd=os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"),
        capture_output=True, text=True)
    last = [l for l in r.stdout.strip().splitlines() if l][-1] if r.stdout else "?"
    return (OK if r.returncode == 0 else FAIL), f"pytest: {last}"


def main():
    global _CAN_OWNER_ERROR, _CAN_OWNER_SNAPSHOT

    print("=" * 72)
    print("  모터 HIL 사전 점검 — 전원 올리기 전에 통과시킬 것")
    if FOUR_WHEEL:
        print("  🛠️ **4륜 모드** — 중륜(ODrive node 13/14) 제외. "
              "그 보드는 부하모터(다이나모)에 쓰는 중.")
    print("=" * 72 + "\n")

    from chassis.runtime_lock import CanOwnershipError, RealCanSession

    can_session = RealCanSession(channel="can0", owner="preflight_hil")
    try:
        can_session.__enter__()
        _CAN_OWNER_SNAPSHOT = can_session.owner_snapshot
        _CAN_OWNER_ERROR = None
    except CanOwnershipError as exc:
        _CAN_OWNER_SNAPSHOT = None
        _CAN_OWNER_ERROR = str(exc)

    try:
        check("CAN 버스", can_up)
        check("좀비 프로세스", no_zombies)
        check("can0 단독 소유권 락", can_lock_owned)
        if _CAN_OWNER_SNAPSHOT is not None:
            check("ODrive 노드 생존 (수동 청취)", odrive_nodes_alive)
        else:
            check(
                "ODrive 노드 생존 (수동 청취)",
                lambda: (FAIL, "owner lock 미획득으로 can0 open을 생략했다"),
            )
        check("기하 — 운용 영역 안전성", geometry_envelope)
        check("기하 변경 경고", geometry_changed_warning)
        check("ODrive 캘리브레이션", calibration_reminder)
        check("코깅존 플로어 (min_rev)", cogging_floor)
        check("단위 테스트", unit_tests)
    finally:
        can_session.close()

    fails = [r for r in _results if r[0] == FAIL]
    warns = [r for r in _results if r[0] == WARN]
    print("\n" + "=" * 72)
    if fails:
        print(f"  {FAIL} 실패 {len(fails)}건 — **HIL 시작하지 말 것**")
        for _, name, _d in fails:
            print(f"     · {name}")
    else:
        print(f"  {OK} 하드 체크 통과. 경고 {len(warns)}건은 읽고 넘어갈 것.")
    print("=" * 72)
    print("""
🛑 **HIL 통과 조건에 반드시 포함할 것**
   · 바퀴를 **완전히 띄운다**. 48V 물리 E-stop 에 손이 닿는 곳에.
   · **텔레메트리만 보고 통과시키지 않는다** — 바퀴 지령 <0.3 rev/s(HALL 코깅존)면
     실물은 정지한 채 텔레메트리만 그럴듯하다. 과거에 실제로 속았다.
     테스트는 **v ≥ 0.4 m/s (바퀴 ≥ 0.6 rev/s)** + **실물 육안 확인**.
   · 첫 선회는 **저속으로**, 조향각이 예상대로 꺾이는지 눈으로 본다(기하가 바뀌었다).
""")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
