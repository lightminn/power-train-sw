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
