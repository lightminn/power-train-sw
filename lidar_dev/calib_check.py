# calib_check.py — 정지 촬영 폴더 하나로 보정 값(높이·각도·거리 오차)을 한 번에 계산하고 기록한다.
# 사용법 (lidar_dev 폴더에서, terrain.py 와 같은 위치):
#   python calib_check.py data/폴더이름 --label "분해 전" --cam-h 0.47 --theta 2 --phone-tilt 17
#   python calib_check.py data/폴더이름 --label "분해 전" --tape 1.0   (상자 앞면까지 줄자 거리, 카메라 앞유리 기준)
# 결과: 화면 출력 + out_seq/calib_log.csv 에 한 줄씩 쌓임 (분해 전/재조립 후 비교용)
import argparse, csv, datetime, glob, os, statistics
import cv2
from terrain import load_detector


def med(v):
    return statistics.median(v) if v else None


def sd(v):
    return statistics.pstdev(v) if len(v) > 1 else 0.0


def fmt(v, nd=3, unit=""):
    return "-" if v is None else f"{v:.{nd}f}{unit}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--label", default="", help="예: 분해 전 / 재조립 후")
    ap.add_argument("--tape", type=float, help="상자 앞면까지 줄자 거리(m). 상자 없는 바닥 촬영이면 생략")
    ap.add_argument("--cam-h", type=float, help="줄자로 잰 카메라 높이(m)")
    ap.add_argument("--theta", type=float, default=0.0, help="차체가 뒤로 기운 각도(도). 뒤로 기울면 +, 앞으로 기울면 -")
    ap.add_argument("--phone-tilt", type=float, help="차체를 수평으로 두고 폰으로 잰 카메라 하향 각도(도)")
    ap.add_argument("--skip", type=int, default=5, help="처음 몇 프레임은 안정화 때문에 제외(기본 5)")
    ap.add_argument("--out", default="out_seq")
    a = ap.parse_args()

    name = os.path.basename(a.folder.rstrip("/"))
    det = load_detector(a.folder)
    files = sorted(glob.glob(os.path.join(a.folder, "depth", "*.png")))
    if not files:
        raise SystemExit("depth 폴더에 이미지가 없습니다: " + a.folder)

    h, t, c, ok = [], [], [], 0
    used = 0
    for i, p in enumerate(files):
        r = det.process(cv2.imread(p, cv2.IMREAD_UNCHANGED))
        if i < a.skip:
            continue
        used += 1
        if r["status"] == "OK":
            ok += 1
            if r["camera_height_m"] is not None:
                h.append(r["camera_height_m"])
                t.append(r["camera_tilt_deg"])
            if r["zones"]["center"] is not None:
                c.append(r["zones"]["center"])

    height, tilt, center = med(h), med(t), med(c)
    level_tilt = None if tilt is None else tilt + a.theta  # 수평 환산 각도
    ok_pct = 100.0 * ok / used if used else 0.0

    print(f"\n=== {name} {('(' + a.label + ')') if a.label else ''} ===")
    print(f"사용 프레임 {used} (앞 {a.skip}개 제외), 바닥 인식 OK {ok_pct:.0f}%")
    print(f"카메라 높이  : {fmt(height, 3, ' m')}  (표준편차 {sd(h)*1000:.1f} mm)")
    if a.cam_h is not None and height is not None:
        print(f"  줄자 {a.cam_h:.3f} m 와 차이 {100*(height-a.cam_h):+.1f} cm")
    print(f"카메라 각도  : {fmt(tilt, 1, '°')}  (표준편차 {sd(t):.2f}°)  ← 크기만 나옴, 방향은 직접 확인")
    print(f"  차체 기울기 θ={a.theta:+.1f}° 반영한 수평 환산: {fmt(level_tilt, 1, '°')}   (설계 하향 15~20°)")
    if a.phone_tilt is not None and level_tilt is not None:
        print(f"  폰으로 잰 {a.phone_tilt:.1f}° 와 차이 {level_tilt - a.phone_tilt:+.1f}°")
    err_cm = None
    if a.tape is not None:
        if center is None:
            print("중앙 거리    : 장애물이 안 잡혔습니다 (상자가 중앙에 있는지, 3.0 m 안쪽인지 확인)")
        else:
            err_cm = 100 * (center - a.tape)
            verdict = "통과" if abs(err_cm) <= 5 else "목표(±5 cm) 초과"
            print(f"중앙 거리    : {center:.2f} m  (프레임 {len(c)}개 중앙값)")
            print(f"  줄자 {a.tape:.2f} m 와 오차 {err_cm:+.1f} cm → {verdict}")
    else:
        print(f"중앙 거리    : {'장애물 없음' if center is None else fmt(center, 2, ' m') + ' (장애물이 보임: 상자 없는 촬영이면 오탐)'}")

    os.makedirs(a.out, exist_ok=True)
    log = os.path.join(a.out, "calib_log.csv")
    new = not os.path.exists(log)
    with open(log, "a", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(["time", "label", "folder", "frames", "ok_pct", "height_m", "height_std_mm", "tilt_deg",
                        "theta_deg", "level_tilt_deg", "tape_cam_h_m", "phone_tilt_deg", "center_m", "tape_dist_m", "err_cm"])
        w.writerow([datetime.datetime.now().strftime("%Y-%m-%d %H:%M"), a.label, name, used, round(ok_pct),
                    "" if height is None else round(height, 3), round(sd(h) * 1000, 1),
                    "" if tilt is None else round(tilt, 1), a.theta,
                    "" if level_tilt is None else round(level_tilt, 1),
                    "" if a.cam_h is None else a.cam_h, "" if a.phone_tilt is None else a.phone_tilt,
                    "" if center is None else round(center, 2), "" if a.tape is None else a.tape,
                    "" if err_cm is None else round(err_cm, 1)])
    print(f"\n기록 저장: {log}")


if __name__ == "__main__":
    main()
