# US-100 충돌방지 안전 모듈 — 구현 계획 (초보자용, 따라하기)

> [!CAUTION]
> **SUPERSEDED — 역사 보존용 문서이며 이 계획을 실행하지 마십시오.**
> 아래 `safe/warn/stop`, `Verdict.level`, 초기 `stop`, `None`→`stop` 구현·시험 단계는
> 폐기되었습니다. 현재 정본은
> [WP5.1 제어·안전 설계](../specs/2026-07-10-wp5-control-safety-hardening-design.md),
> [WP5.1 구현 계획](./2026-07-10-wp5-control-safety-hardening-plan.md),
> [실행 모듈 README](../../motor_control/safety_us100/README.md)입니다. 아래 Task와 명령은
> 2026-05-25 당시 기록을 보존할 뿐이므로 복사·실행하거나 현재 API로 해석하지 마십시오.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** US-100 거리 센서 1개로 앞쪽 장애물 거리를 재서 "안전/주의/멈춤"을 알려주는(직접 멈추지는 않는) 작은 프로그램을 만든다.

**Architecture:** "거리 → 판정" 계산(`evaluator`), "센서에서 거리 읽기"(`us100`), "둘을 합쳐 계속 감시"(`safety_monitor`)를 파일로 나눈다. 센서가 없어도 되는 계산 부분은 가짜 센서로 자동 시험하고, 진짜 센서를 읽는 부분은 로봇에 연결해 눈으로 확인한다.

**Tech Stack:** Python 3, `pyserial`(센서 통신), `pytest`(자동 시험). 설계 문서: `docs/specs/2026-05-25-us100-safety-module-design.md` (먼저 읽어 주세요).

---

## 시작하기 전에 (초보자 안내)

이 계획은 **위에서부터 차례대로** 따라 하면 됩니다. 각 작업(Task)은 보통 이런 흐름입니다:

1. **시험 코드를 먼저 쓴다** (아직 프로그램이 없어서 실패하는 게 정상)
2. **시험을 돌려 실패를 확인한다** (왜 먼저 실패시키냐면, 시험이 진짜 동작하는지 믿으려고)
3. **프로그램을 만든다**
4. **시험을 다시 돌려 통과를 확인한다**
5. **저장한다 (git commit)**

알아두면 좋은 것:
- **터미널 명령어**는 회색 칸에 있는 걸 그대로 복사해 붙여 넣으면 됩니다.
- **`pytest`** 는 "자동 시험 도구"입니다. 우리가 쓴 시험 코드를 자동으로 실행해 맞는지 알려줍니다.
- **`git commit`** 은 "지금까지 한 일을 저장(스냅샷)"하는 것입니다. 자주 저장하면 안전합니다.
- 모든 명령어는 `motor_control` 폴더에서 실행한다고 가정합니다. 명령 앞의
  `cd /home/light/Defence_Robot/motor_control` 가 그 폴더로 이동하는 명령입니다.
- 파이썬 패키지 설치(`pyserial`)가 필요하면 x86 dev 컨테이너(`powertrain_dev`) 안에서
  하고, 빠져 있으면 `docker/Dockerfile` 에 추가합니다. (자동 시험 Task 1~4는 `pyserial`
  없이도 돕니다. `pyserial` 은 진짜 센서를 읽는 Task 5에서만 필요합니다.)

용어가 헷갈리면 설계 문서 3절(핵심 개념)을 다시 보세요.

---

## 만들 파일 목록 (File Structure)

```
motor_control/safety_us100/
├── __init__.py          # "이 폴더는 파이썬 묶음이에요" 표시용 빈 파일   [Task 1]
├── config.py            # 설정값 모음 (거리 기준 등)                      [Task 1]
├── verdict.py           # 판정 결과 모양 (단계 + 거리)                    [Task 1]
├── evaluator.py         # 거리 → 판정 계산 (순수 계산)                    [Task 2]
├── fake_sensor.py       # 시험용 가짜 센서                                [Task 3]
├── safety_monitor.py    # 감시 본체 (tick/verdict + 모르면 멈춤)          [Task 4]
├── us100.py             # 진짜 센서 읽기 (하드웨어)                       [Task 5]
├── demo.py              # 직접 켜서 눈으로 확인                           [Task 6]
├── README.md            # 사용 설명서                                     [Task 6]
└── tests/
    └── test_safety.py   # 자동 시험 모음                            [Task 1~4]
```

---

## Task 1: 폴더 만들기 + 설정값(config) + 판정 모양(verdict)

**Files:**
- Create: `motor_control/safety_us100/__init__.py` (빈 파일)
- Create: `motor_control/safety_us100/config.py`
- Create: `motor_control/safety_us100/verdict.py`
- Create: `motor_control/safety_us100/tests/test_safety.py`

- [ ] **Step 1: 빈 표시 파일 만들기**

`motor_control/safety_us100/__init__.py` 를 **내용 없이** 만든다. (파이썬에게 "이 폴더는
묶음(패키지)이에요"라고 알려주는 표시일 뿐입니다.)

- [ ] **Step 2: 실패하는 시험 코드 쓰기**

`motor_control/safety_us100/tests/test_safety.py`:

```python
from safety_us100.verdict import Verdict, SAFE, WARN, STOP
from safety_us100.config import SafetyConfig


def test_level_names():
    # 단계 이름이 정해진 글자와 같은지 확인
    assert SAFE == "safe"
    assert WARN == "warn"
    assert STOP == "stop"


def test_verdict_holds_level_and_distance():
    # 판정 결과는 "단계"와 "거리" 두 가지를 담는다
    v = Verdict(level=SAFE, distance_mm=500.0)
    assert v.level == "safe"
    assert v.distance_mm == 500.0


def test_config_default_values():
    c = SafetyConfig()
    assert c.warn_mm == 400.0
    assert c.stop_mm == 200.0
    assert c.hysteresis_mm == 30.0
    assert c.fail_stop_count == 3
    assert c.port == "/dev/ttyTHS1"
    assert c.baud == 9600
```

- [ ] **Step 3: 시험을 돌려 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 실패 — `ModuleNotFoundError: No module named 'safety_us100.verdict'` (아직 안 만들어서 정상)

- [ ] **Step 4: verdict.py 만들기**

`motor_control/safety_us100/verdict.py`:

```python
"""판정 결과의 모양을 정해 둔 파일.

판정은 딱 두 가지만 담는다: 단계(safe/warn/stop)와 거리(mm).
"""
from dataclasses import dataclass
from typing import Optional

# 단계 이름. 글자를 직접 쓰면 오타가 나기 쉬워서 미리 정해 둔다.
SAFE = "safe"
WARN = "warn"
STOP = "stop"


@dataclass(frozen=True)
class Verdict:
    level: str                    # "safe" / "warn" / "stop"
    distance_mm: Optional[float]  # 잰 거리(mm). 못 쟀으면 None(비어 있음)
```

- [ ] **Step 5: config.py 만들기**

`motor_control/safety_us100/config.py`:

```python
"""바꿀 수 있는 설정값을 한곳에 모은 파일.

거리 기준이나 여유 값을 조절하고 싶으면 여기 숫자만 고치면 된다.
"""
from dataclasses import dataclass


@dataclass
class SafetyConfig:
    warn_mm: float = 400.0       # 이 거리(mm) 이하면 "주의" (=40cm)
    stop_mm: float = 200.0       # 이 거리(mm) 이하면 "멈춤" (=20cm)
    hysteresis_mm: float = 30.0  # 깜빡임 방지 여유 (=3cm)
    fail_stop_count: int = 3     # 연속 몇 번 못 읽으면 "멈춤"으로 볼지
    port: str = "/dev/ttyTHS1"   # 센서가 연결된 선 이름 (그대로 두기)
    baud: int = 9600             # 통신 속도 (그대로 두기)
```

- [ ] **Step 6: 시험을 돌려 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 통과 (3 passed)

- [ ] **Step 7: 저장(commit)**

```bash
cd /home/light/Defence_Robot
git add motor_control/safety_us100/__init__.py motor_control/safety_us100/config.py motor_control/safety_us100/verdict.py motor_control/safety_us100/tests/test_safety.py
git commit -m "feat(safety_us100): 설정값(config) + 판정 모양(verdict) + 폴더 만들기"
```

---

## Task 2: 거리 → 판정 계산 (evaluator)

**Files:**
- Create: `motor_control/safety_us100/evaluator.py`
- Modify: `motor_control/safety_us100/tests/test_safety.py` (시험 추가)

설계 문서 6절의 규칙을 그대로 코드로 옮긴다. 이 파일은 **센서 없이** 숫자만으로
동작하므로 시험하기 가장 쉽다.

- [ ] **Step 1: 실패하는 시험 코드 쓰기** (파일 맨 끝에 추가)

```python
from safety_us100.evaluator import evaluate


def test_far_distance_is_safe():
    cfg = SafetyConfig()
    assert evaluate(500.0, cfg, prev_level=None) == SAFE


def test_mid_distance_is_warn():
    cfg = SafetyConfig()
    assert evaluate(300.0, cfg, prev_level=None) == WARN


def test_near_distance_is_stop():
    cfg = SafetyConfig()
    assert evaluate(150.0, cfg, prev_level=None) == STOP


def test_no_reading_is_stop():
    # 거리를 못 쟀으면(None) "모르면 멈춤"
    cfg = SafetyConfig()
    assert evaluate(None, cfg, prev_level=None) == STOP


def test_escalation_is_immediate():
    # 더 위험해질 때는 망설임 없이 바로 바뀐다
    cfg = SafetyConfig()
    assert evaluate(150.0, cfg, prev_level=SAFE) == STOP


def test_hysteresis_holds_stop_near_threshold():
    # "멈춤"이었는데 거리가 살짝(210mm)만 멀어지면 아직 "멈춤" 유지
    cfg = SafetyConfig()  # stop=200, 여유=30 → 230 넘어야 풀림
    assert evaluate(210.0, cfg, prev_level=STOP) == STOP


def test_release_from_stop_after_margin():
    # 충분히(250mm) 멀어지면 "멈춤"에서 풀려 "주의"로
    cfg = SafetyConfig()
    assert evaluate(250.0, cfg, prev_level=STOP) == WARN


def test_hysteresis_holds_warn_near_threshold():
    # "주의"였는데 거리가 살짝(410mm)만 멀어지면 아직 "주의" 유지
    cfg = SafetyConfig()  # warn=400, 여유=30 → 430 넘어야 풀림
    assert evaluate(410.0, cfg, prev_level=WARN) == WARN
    # 충분히(440mm) 멀어지면 "안전"으로
    assert evaluate(440.0, cfg, prev_level=WARN) == SAFE
```

- [ ] **Step 2: 시험을 돌려 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 실패 — `ModuleNotFoundError: No module named 'safety_us100.evaluator'`

- [ ] **Step 3: evaluator.py 만들기**

`motor_control/safety_us100/evaluator.py`:

```python
"""거리 숫자를 받아 단계(safe/warn/stop)를 계산하는 부분.

규칙은 설계 문서 6절과 같다:
  - 거리를 못 쟀으면(None) → 멈춤 (모르면 위험)
  - 멈춤 기준 이하 → 멈춤 / 주의 기준 이하 → 주의 / 그 외 → 안전
  - 깜빡임 방지: 더 안전한 쪽으로 풀 때만 '여유'만큼 더 멀어져야 함
"""
from safety_us100.verdict import SAFE, WARN, STOP

# 위험한 정도에 점수를 매긴다(클수록 위험). 비교에 쓴다.
_DANGER = {SAFE: 0, WARN: 1, STOP: 2}


def evaluate(distance_mm, cfg, prev_level):
    # 1) 거리를 못 쟀으면 무조건 멈춤
    if distance_mm is None:
        return STOP

    # 2) 거리만 보고 단계를 정한다 (여유는 아직 적용 전)
    if distance_mm <= cfg.stop_mm:
        raw = STOP
    elif distance_mm <= cfg.warn_mm:
        raw = WARN
    else:
        raw = SAFE

    # 3) 직전 단계가 없으면 그대로 사용
    if prev_level is None:
        return raw

    # 4) 더 위험해지거나 같은 단계면 즉시 적용 (안전 우선)
    if _DANGER[raw] >= _DANGER[prev_level]:
        return raw

    # 5) 더 안전한 쪽으로 푸는 경우 → '여유'만큼 더 멀어져야 한 단계씩 풀림
    if prev_level == STOP:
        if distance_mm <= cfg.stop_mm + cfg.hysteresis_mm:
            return STOP   # 아직 멈춤 유지
        if distance_mm <= cfg.warn_mm + cfg.hysteresis_mm:
            return WARN
        return SAFE
    if prev_level == WARN:
        if distance_mm <= cfg.warn_mm + cfg.hysteresis_mm:
            return WARN   # 아직 주의 유지
        return SAFE

    return raw
```

- [ ] **Step 4: 시험을 돌려 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 통과 (11 passed)

- [ ] **Step 5: 저장(commit)**

```bash
cd /home/light/Defence_Robot
git add motor_control/safety_us100/evaluator.py motor_control/safety_us100/tests/test_safety.py
git commit -m "feat(safety_us100): 거리 → 판정 계산 + 깜빡임 방지"
```

---

## Task 3: 시험용 가짜 센서 (fake_sensor)

**Files:**
- Create: `motor_control/safety_us100/fake_sensor.py`
- Modify: `motor_control/safety_us100/tests/test_safety.py`

진짜 센서가 없어도 본체를 시험하려면, "내가 정한 거리들을 차례로 내놓는" 가짜 센서가
필요하다. 진짜 센서(`us100.py`)와 똑같이 `read()` 라는 사용법을 갖게 만든다.

- [ ] **Step 1: 실패하는 시험 코드 쓰기** (파일 맨 끝에 추가)

```python
from safety_us100.fake_sensor import FakeUs100


def test_fake_returns_readings_in_order():
    s = FakeUs100([500.0, 300.0, 150.0])
    assert s.read() == 500.0
    assert s.read() == 300.0
    assert s.read() == 150.0


def test_fake_repeats_last_after_end():
    # 정해둔 값을 다 쓰면 마지막 값을 계속 내놓는다
    s = FakeUs100([400.0])
    s.read()
    assert s.read() == 400.0
    assert s.read() == 400.0


def test_fake_can_return_none():
    # None(못 쟀음)도 흉내낼 수 있다
    s = FakeUs100([None])
    assert s.read() is None
```

- [ ] **Step 2: 시험을 돌려 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 실패 — `ModuleNotFoundError: No module named 'safety_us100.fake_sensor'`

- [ ] **Step 3: fake_sensor.py 만들기**

`motor_control/safety_us100/fake_sensor.py`:

```python
"""시험용 가짜 센서.

미리 정한 거리 목록을 read() 할 때마다 하나씩 내놓는다. 목록을 다 쓰면
마지막 값을 계속 내놓는다. None 을 넣으면 '못 쟀음'을 흉내낼 수 있다.
"""


class FakeUs100:
    def __init__(self, readings):
        self._readings = list(readings)
        self._index = 0

    def read(self):
        if not self._readings:
            return None
        if self._index < len(self._readings):
            value = self._readings[self._index]
            self._index += 1
            return value
        return self._readings[-1]  # 다 쓰면 마지막 값 반복
```

- [ ] **Step 4: 시험을 돌려 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 통과 (14 passed)

- [ ] **Step 5: 저장(commit)**

```bash
cd /home/light/Defence_Robot
git add motor_control/safety_us100/fake_sensor.py motor_control/safety_us100/tests/test_safety.py
git commit -m "feat(safety_us100): 시험용 가짜 센서"
```

---

## Task 4: 감시 본체 (safety_monitor)

**Files:**
- Create: `motor_control/safety_us100/safety_monitor.py`
- Modify: `motor_control/safety_us100/tests/test_safety.py`

센서에서 읽기 → 판정 계산 → 보관을 반복하는 본체. "연속으로 못 읽으면 멈춤" 처리도
여기서 한다. 켜자마자(아직 한 번도 못 읽었을 때)는 안전하게 "멈춤"으로 시작한다.

- [ ] **Step 1: 실패하는 시험 코드 쓰기** (파일 맨 끝에 추가)

```python
from safety_us100.safety_monitor import SafetyMonitor


def test_initial_verdict_is_stop():
    # 아직 한 번도 안 읽었으면 안전하게 "멈춤"으로 시작
    mon = SafetyMonitor(FakeUs100([500.0]), SafetyConfig())
    assert mon.verdict().level == STOP


def test_far_reading_gives_safe():
    mon = SafetyMonitor(FakeUs100([500.0]), SafetyConfig())
    mon.tick()
    assert mon.verdict().level == SAFE
    assert mon.verdict().distance_mm == 500.0


def test_near_reading_gives_stop():
    mon = SafetyMonitor(FakeUs100([150.0]), SafetyConfig())
    mon.tick()
    assert mon.verdict().level == STOP


def test_transient_failures_keep_previous():
    # 멀리(safe) 읽은 뒤 한두 번 실패는 직전 판정(safe) 유지
    mon = SafetyMonitor(FakeUs100([500.0, None, None, None]), SafetyConfig(fail_stop_count=3))
    mon.tick()   # 500 → safe, 실패횟수 0
    assert mon.verdict().level == SAFE
    mon.tick()   # None → 실패 1 (< 3) → 유지
    assert mon.verdict().level == SAFE
    mon.tick()   # None → 실패 2 (< 3) → 유지
    assert mon.verdict().level == SAFE


def test_persistent_failure_gives_stop():
    mon = SafetyMonitor(FakeUs100([500.0, None, None, None]), SafetyConfig(fail_stop_count=3))
    mon.tick()   # safe
    mon.tick()   # 실패 1
    mon.tick()   # 실패 2
    mon.tick()   # 실패 3 → 멈춤
    assert mon.verdict().level == STOP
    assert mon.verdict().distance_mm is None


def test_recovery_to_safe():
    # 가까웠다(stop)가 충분히 멀어지면 다시 safe
    mon = SafetyMonitor(FakeUs100([150.0, 500.0]), SafetyConfig())
    mon.tick()   # 150 → stop
    assert mon.verdict().level == STOP
    mon.tick()   # 500 → safe
    assert mon.verdict().level == SAFE


def test_no_chatter_with_hysteresis():
    # 멈춤 기준(200) 근처에서 거리가 흔들려도 "멈춤"으로 안정
    mon = SafetyMonitor(FakeUs100([150.0, 210.0, 190.0, 210.0]), SafetyConfig())
    levels = []
    for _ in range(4):
        mon.tick()
        levels.append(mon.verdict().level)
    assert levels == [STOP, STOP, STOP, STOP]


def test_verdict_schema():
    mon = SafetyMonitor(FakeUs100([500.0]), SafetyConfig())
    mon.tick()
    v = mon.verdict()
    assert hasattr(v, "level")
    assert hasattr(v, "distance_mm")
```

- [ ] **Step 2: 시험을 돌려 실패 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 실패 — `ModuleNotFoundError: No module named 'safety_us100.safety_monitor'`

- [ ] **Step 3: safety_monitor.py 만들기**

`motor_control/safety_us100/safety_monitor.py`:

```python
"""감시 본체.

tick() 을 부를 때마다 센서를 한 번 읽어 판정을 갱신한다.
- 잘 읽히면: 판정 계산 + 실패횟수 초기화
- 못 읽으면: 실패횟수 증가. 연속 실패가 기준에 도달하면 '멈춤'으로 (모르면 위험)
verdict() 로 최신 판정을 꺼낸다.
"""
from safety_us100.verdict import Verdict, STOP
from safety_us100.evaluator import evaluate


class SafetyMonitor:
    def __init__(self, sensor, cfg):
        self._sensor = sensor
        self._cfg = cfg
        self._fail_count = 0
        # 아직 아무것도 못 읽었으니 안전하게 멈춤으로 시작
        self._verdict = Verdict(level=STOP, distance_mm=None)

    def tick(self):
        distance = self._sensor.read()

        if distance is None:
            # 못 읽음: 실패 횟수를 센다
            self._fail_count += 1
            if self._fail_count >= self._cfg.fail_stop_count:
                self._verdict = Verdict(level=STOP, distance_mm=None)
            # 아직 기준 미만이면 직전 판정을 그대로 둔다
            return

        # 잘 읽힘: 실패 횟수 초기화하고 판정 계산
        self._fail_count = 0
        level = evaluate(distance, self._cfg, self._verdict.level)
        self._verdict = Verdict(level=level, distance_mm=distance)

    def verdict(self):
        return self._verdict
```

- [ ] **Step 4: 시험을 돌려 통과 확인**

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 통과 (22 passed)

- [ ] **Step 5: 저장(commit)**

```bash
cd /home/light/Defence_Robot
git add motor_control/safety_us100/safety_monitor.py motor_control/safety_us100/tests/test_safety.py
git commit -m "feat(safety_us100): 감시 본체 (tick/verdict + 모르면 멈춤)"
```

---

## Task 5: 진짜 센서 읽기 (us100) — 하드웨어 필요

**Files:**
- Create: `motor_control/safety_us100/us100.py`

이 부분만 진짜 부품(US-100 센서 + 로봇)이 있어야 시험할 수 있다. 그래서 자동 시험
(pytest) 대신 **로봇에 연결해 눈으로 확인**한다. 읽는 방법은 이미 있는
`motor_control/sensors/us100_robust.py` 를 그대로 따른다.

- [ ] **Step 1: us100.py 만들기**

`motor_control/safety_us100/us100.py`:

```python
"""진짜 US-100 센서에서 거리를 읽어오는 부분.

읽는 요령은 motor_control/sensors/us100_robust.py 와 같다:
센서에게 0x55(재 줘) 신호를 보내되, Jetson 버그를 피하려고 앞에 0xFF 더미를
몇 개 붙인다. 답(2바이트)은 받은 데이터의 끝 2바이트에서 꺼낸다.
read() 는 거리를 mm 로 돌려주고, 못 읽으면 None 을 돌려준다.
"""
import time

import serial  # pyserial 패키지. 없으면 dev 컨테이너에서 설치


class Us100Sensor:
    def __init__(self, port="/dev/ttyTHS1", baud=9600, timeout=0.1):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._ser = None

    def open(self):
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
        except serial.SerialException as e:
            raise RuntimeError(
                f"센서 포트({self._port})를 열 수 없습니다. 연결과 권한을 확인하세요. ({e})"
            ) from e

    def read(self):
        if self._ser is None:
            return None
        try:
            self._ser.reset_input_buffer()
            # 0xFF 더미 8개 + 0x55(재 줘) — Jetson 첫 글자 깨짐 버그 우회
            self._ser.write(b"\xff" * 8 + bytes([0x55]))
            time.sleep(0.1)
            data = self._ser.read(64)
            if len(data) < 2:
                return None
            high, low = data[-2], data[-1]   # 끝 2바이트가 답
            mm = high * 256 + low
            if 20 <= mm <= 4000:             # 유효 범위(2cm~4m) 안일 때만
                return float(mm)
            return None
        except serial.SerialException:
            return None

    def close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None
```

- [ ] **Step 2: 불러오기만 확인 (부품 없이)**

`pyserial` 이 설치된 환경(dev 컨테이너)에서:
Run: `cd /home/light/Defence_Robot/motor_control && python -c "import sys; sys.path.insert(0,'.'); from safety_us100.us100 import Us100Sensor; print('불러오기 성공')"`
Expected: `불러오기 성공`
(만약 `ModuleNotFoundError: No module named 'serial'` 이 나오면 `pyserial` 설치 필요:
dev 컨테이너에서 `pip install pyserial`, 그리고 `docker/Dockerfile` 에 추가.)

- [ ] **Step 3: 로봇에서 직접 확인 (진짜 센서)**

센서를 로봇(Jetson)에 연결한 뒤:
```bash
cd /home/light/Defence_Robot/motor_control
python3 -c "
import sys; sys.path.insert(0,'.')
from safety_us100.us100 import Us100Sensor
import time
s = Us100Sensor()
s.open()
for _ in range(20):
    print('거리:', s.read(), 'mm')
    time.sleep(0.2)
s.close()
"
```
Expected: 손을 센서 앞에서 멀리/가까이 움직이면 숫자가 그에 맞게 커지고 작아진다.
가끔 `None` 이 섞여 나올 수 있다(정상 — 본체가 알아서 처리함).

- [ ] **Step 4: 저장(commit)**

```bash
cd /home/light/Defence_Robot
git add motor_control/safety_us100/us100.py
git commit -m "feat(safety_us100): 진짜 US-100 센서 읽기 (하드웨어)"
```

---

## Task 6: 직접 켜보는 프로그램(demo) + 설명서(README)

**Files:**
- Create: `motor_control/safety_us100/demo.py`
- Create: `motor_control/safety_us100/README.md`

- [ ] **Step 1: demo.py 만들기**

`motor_control/safety_us100/demo.py`:

```python
"""직접 켜서 눈으로 확인하는 프로그램.

센서를 열고, 0.1초마다 거리를 재서 판정과 함께 화면에 찍는다.
거리 기준이 적당한지 보고 config.py 값을 조절하는 데 쓴다.
Ctrl-C 를 누르면 센서를 정리하고 끝난다.
"""
import sys
import time


def main():
    sys.path.insert(0, ".")
    from safety_us100.config import SafetyConfig
    from safety_us100.us100 import Us100Sensor
    from safety_us100.safety_monitor import SafetyMonitor

    cfg = SafetyConfig()
    sensor = Us100Sensor(port=cfg.port, baud=cfg.baud)
    sensor.open()
    monitor = SafetyMonitor(sensor, cfg)

    print("US-100 충돌방지 데모 시작. 끝내려면 Ctrl-C.")
    try:
        while True:
            monitor.tick()
            v = monitor.verdict()
            shown = "(없음)" if v.distance_mm is None else f"{int(v.distance_mm)} mm"
            print(f"거리: {shown}\t판정: {v.level}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
        print("\n종료했습니다.")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 자동 시험이 여전히 통과하는지 확인** (demo 추가가 기존 걸 깨지 않았는지)

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/test_safety.py -v`
Expected: 통과 (22 passed)

- [ ] **Step 3: 로봇에서 데모 실행 (진짜 센서)**

```bash
cd /home/light/Defence_Robot/motor_control
python3 safety_us100/demo.py
```
Expected: 화면에 `거리: 512 mm    판정: safe` 처럼 계속 찍힌다. 손을 가까이 가져가면
`warn` → `stop` 으로, 멀리하면 다시 `safe` 로 바뀐다. 선을 뽑으면 잠시 뒤 `stop (없음)`.
`Ctrl-C` 로 종료. 거리 기준이 마음에 안 들면 `config.py` 의 `warn_mm`/`stop_mm` 을 고친다.

- [ ] **Step 4: README.md 만들기**

`motor_control/safety_us100/README.md`:

````markdown
# safety_us100 — US-100 충돌방지 안전 모듈

앞쪽 거리 센서(US-100) 하나로 장애물이 얼마나 가까운지 재서 "안전/주의/멈춤"을
알려주는 프로그램입니다. **모터를 직접 멈추지는 않고 알려주기만** 합니다.
자세한 배경은 `docs/specs/2026-05-25-us100-safety-module-design.md` 를 보세요.

## 파일 설명
- `config.py` — 거리 기준 등 설정값 (여기 숫자만 고치면 됨)
- `verdict.py` — 판정 결과 모양 (단계 + 거리)
- `evaluator.py` — 거리 → 단계 계산
- `safety_monitor.py` — 계속 감시하는 본체
- `us100.py` — 진짜 센서에서 거리 읽기
- `fake_sensor.py` — 시험용 가짜 센서
- `demo.py` — 직접 켜서 확인하는 프로그램

## 자동 시험 (부품 없이, dev 컨테이너에서)
```bash
cd /home/light/Defence_Robot/motor_control
python -m pytest safety_us100/tests/ -v
```

## 로봇에서 실제로 돌려보기 (센서 연결 후)
```bash
cd /home/light/Defence_Robot/motor_control
python3 safety_us100/demo.py
```
손을 센서 앞에서 움직여 단계가 바뀌는지 확인하고, `config.py` 로 거리 기준을 조절하세요.

## 다른 프로그램에서 결과 쓰는 법 (예시)
```python
monitor.tick()                          # 한 번 확인
if monitor.verdict().level == "stop":
    전진_속도 = 0                       # 앞으로 가는 명령을 0으로
```

## 지금은 안 만든 것 (나중 과제)
- 추락방지(바닥 향한 센서), 센서 여러 개, 모터 직접 멈춤 — 모두 제외.
````

- [ ] **Step 5: 저장(commit)**

```bash
cd /home/light/Defence_Robot
git add motor_control/safety_us100/demo.py motor_control/safety_us100/README.md
git commit -m "feat(safety_us100): 데모 프로그램 + 사용 설명서"
```

---

## 전체 확인 (모든 작업 끝난 뒤)

Run: `cd /home/light/Defence_Robot/motor_control && python -m pytest safety_us100/tests/ -v`
Expected: 통과 (22 passed) — 계산·본체 부분 전부. 진짜 센서를 읽는 부분(Task 5,6)은
로봇에 연결해 데모로 눈으로 확인한다.
