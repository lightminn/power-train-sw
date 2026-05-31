# safety_us100 — US-100 충돌방지 안전 모듈

앞쪽 거리 센서(US-100) 하나로 장애물이 얼마나 가까운지 재서 "안전/주의/멈춤"을
알려주는 프로그램입니다. 모터를 직접 멈추지는 않고 알려주기만 합니다.

## 파일 설명
- `config.py` — 거리 기준 등 설정값
- `verdict.py` — 판정 결과 모양 (단계 + 거리)
- `evaluator.py` — 거리 → 단계 계산
- `safety_monitor.py` — 계속 감시하는 본체
- `us100.py` — 진짜 센서에서 거리 읽기
- `fake_sensor.py` — 시험용 가짜 센서
- `demo.py` — 직접 켜서 확인하는 프로그램

## 자동 시험 (도커 안에서)
```bash
cd /workspace/motor_control
python -m pytest safety_us100/tests/ -v
```

## 데모 실행 (센서 연결 후)
```bash
python3 safety_us100/demo.py
```

## 다른 프로그램에서 쓰는 법
```python
monitor.tick()
if monitor.verdict().level == "stop":
    전진_속도 = 0
```
