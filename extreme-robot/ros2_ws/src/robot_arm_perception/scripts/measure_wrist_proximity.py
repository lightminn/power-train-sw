#!/usr/bin/env python3
"""손목 캠 거리 곡선 실측 — `f_px` 와 기준점 오프셋을 구한다 (2026-08-13 신설).

## 왜 필요한가

`wrist_camera` 의 `metrics.distance_m` 은 `f_px` 가 0 인 동안 **항상 `null`** 이다.
하강 종료 판단(2단계)이 이 값에 걸려 있으므로, 그 전에 한 번은 실측해야 한다.

## 무엇을 재는가 — 가로 폭만이다

2026-08-13 1차 확인에서 같은 프레임의 같은 상자로 뽑은 `f_px` 가 **가로 412 / 세로 150**
으로 2.7배 어긋났다. 손목 캠이 상자를 비스듬히 내려다봐 **세로만 단축**되기 때문이다.
그래서 이 스크립트는 `bbox_w` 만 쓴다.

## 점을 여러 개 재는 이유 (한 점이면 안 되는 이유)

`f_px = w*d/S` 는 `d` 가 **렌즈 주점**에서 잰 거리일 때만 맞다. 자로 재는 실제 기준점은
케이스 앞면 어딘가라 늘 몇 cm 씩 어긋나 있고, 그 오프셋이 한 점 계산에서는 통째로
`f_px` 오차로 둔갑한다 — 가까울수록 크게 틀리는데 하필 거기가 파지 구간이다. 여러 점을
`d = a/w + b` 직선으로 맞추면 `a` 에서 `f_px`, `b` 에서 그 오프셋이 따로 나온다.

## 쓰는 법

손목 캠 노드를 먼저 띄운다(거리 환산은 아직 꺼진 채여도 된다):

    ros2 launch robot_arm_perception wrist_camera.launch.py

다른 터미널에서:

    source /root/ros2_ws/install/setup.bash
    python3 src/robot_arm_perception/scripts/measure_wrist_proximity.py --size 0.095

상자를 **자로 잰 거리**에 두고 그 거리를 mm 로 입력 → Enter. 4~6개 점을 파지 거리
전후로 고루 잡는다(가까운 쪽을 촘촘히). 빈 줄이면 계산·저장하고 끝난다.

⚠️ **매번 같은 기준점에서 재야 한다**(예: 카메라 케이스 앞면 ↔ 상자 앞면). 기준점이
왔다갔다 하면 절편이 아니라 **잡음**이 되어 `f_px` 까지 흔든다.
⚠️ 상자가 화면에서 잘리는 거리(두 변 이상 접촉)는 자동으로 버린다 — 그 구간은 겉보기
크기가 거리 신호가 아니다. 파지 직전 거리를 재고 싶으면 카메라를 옆으로 못 옮기므로
**상자를 살짝 뒤로 빼서** 잘리지 않는 가장 가까운 점을 잡을 것.
"""
import argparse
import json
import os
import sys
import time

import rclpy

try:
    from robot_arm_perception import wrist_metrics
    from robot_arm_perception.wrist_metrics import usable
    from robot_arm_perception.wrist_sampling import (
        MetricsCollector, shutdown, spin_in_background,
    )
except ImportError:
    sys.stderr.write(
        'robot_arm_perception 을 import 할 수 없습니다 — 워크스페이스 오버레이가\n'
        '소싱되지 않은 셸로 보입니다. 다음을 먼저 실행하세요:\n\n'
        '    source /root/ros2_ws/install/setup.bash\n')
    sys.exit(1)


def _collect_point(collector, distance_m, count):
    """한 거리에서 표본을 모아 `(중앙값 폭, σ, 통계)` 로 줄인다."""
    print(f'  수집 중… ({count}표본)', end='', flush=True)
    samples, timed_out = collector.collect(count)
    print()
    if timed_out:
        print(f'  ⚠️ 타임아웃 — {len(samples)}개만 받았습니다. '
              f'노드가 떠 있는지 확인하세요(ros2 topic hz {collector.topic}).')
    good = [s for s in samples if usable(s)]
    rejected = {
        '미검출': sum(1 for s in samples if not s.get('detected')),
        '가림': sum(1 for s in samples if s.get('detected') and s.get('occluded')),
        '잘림': sum(1 for s in samples
                  if s.get('detected') and int(s.get('border_contacts', 0)) >= 2),
        # 잘라낸 '비율'로 센다 — 실기에서는 케이블이 없어도 가장자리에서 2~3% 는 늘
        # 잘려 나간다(wrist_metrics.DEFAULT_MAX_TRIM_RATIO 주석의 실측 근거).
        '잔가지': sum(1 for s in samples
                   if s.get('detected') and not usable(s, allow_clipped=True)
                   and int(s.get('border_contacts', 0)) < 2 and not s.get('occluded')),
    }
    widths = [float(s['bbox_w']) for s in good if s.get('bbox_w')]
    median, sigma = wrist_metrics.robust_stats(widths)
    return {
        'distance_m': distance_m,
        'bbox_w_median': median,
        'bbox_w_sigma': sigma,
        'n_used': len(widths),
        'n_total': len(samples),
        'rejected': rejected,
        'raw_widths': widths,
    }


def _print_point(point):
    if point['bbox_w_median'] is None:
        print(f'  ✗ 쓸 만한 표본이 0개입니다 — 버림 내역 {point["rejected"]}')
        return
    ratio = point['bbox_w_sigma'] / point['bbox_w_median'] if point['bbox_w_median'] else 0.0
    print(f'  가로폭 중앙값 {point["bbox_w_median"]:.1f}px  σ={point["bbox_w_sigma"]:.1f}px '
          f'({ratio * 100:.1f}%)  사용 {point["n_used"]}/{point["n_total"]}')
    if point['rejected']['잔가지']:
        print(f'  ⚠️ {point["rejected"]["잔가지"]}개 프레임에서 덩어리의 10% 넘게 잘라냈습니다 — '
              '케이블이 계속 붙으면 ROI 를 좁히거나 thin_reject_px 를 올리세요.')
    if ratio > 0.05:
        print('  ⚠️ 산포가 5% 를 넘습니다. 이 점은 곡선을 휘게 합니다 — '
              '조명·자세를 고정하고 다시 재는 편이 낫습니다.')


def _report(points, size_m, out_path):
    usable_points = [(p['distance_m'], p['bbox_w_median'])
                     for p in points if p['bbox_w_median']]
    print('\n' + '=' * 62)
    print(f'{"거리(m)":>10} {"가로폭(px)":>12} {"σ":>7} {"단순 f_px":>10}')
    for p in points:
        if p['bbox_w_median'] is None:
            continue
        naive = p['bbox_w_median'] * p['distance_m'] / size_m
        print(f'{p["distance_m"]:>10.3f} {p["bbox_w_median"]:>12.1f} '
              f'{p["bbox_w_sigma"]:>7.1f} {naive:>10.1f}')

    fit = wrist_metrics.fit_distance_curve(usable_points, real_size_m=size_m)
    print('-' * 62)
    if fit['f_px'] is None:
        print('점이 2개 미만이거나 겉보기 크기가 모두 같아 직선을 못 맞췄습니다.')
        print('⚠️ 위 "단순 f_px" 는 기준점 오프셋이 섞인 값입니다 — 점을 더 재세요.')
    elif not fit['plausible']:
        # ⚠️ 여기서 값을 찍지 않는 게 이 분기의 존재 이유다. 겉보기 크기가 거의 같은
        # 점들만 주면 기울기가 잡음으로 정해져 **형태만 멀쩡한 쓰레기**가 나온다
        # (실제로 f_px=47198 / 오프셋 -20.9m 이 나왔다). 그걸 '적용하세요'로 출력하면
        # 다음 사람은 그 값을 launch 에 넣는다.
        print(f'✗ 이 표본으로는 f_px 를 확정할 수 없습니다: {fit["reason"]}')
        print(f'  (겉보기 크기 범위 {fit["width_span"]:.2f}배, 계산상 f_px={fit["f_px"]:.0f} — '
              '믿지 마세요)')
        print('  권장: 가장 가까운 점의 **2배 거리**까지 4~6점을 고루 잡을 것.')
    else:
        print(f'f_px(가로) = {fit["f_px"]:.1f} px')
        print(f'기준점 오프셋 = {fit["offset_m"] * 1000:.0f} mm '
              '(자로 잰 기준점과 렌즈 주점의 차이. 몇 cm 면 정상)')
        worst = max(abs(r) for r in fit['residuals'])
        print(f'잔차 최대 = {worst * 1000:.1f} mm  (점 {fit["n"]}개)')
        if worst > 0.02:
            print('⚠️ 잔차가 2cm 를 넘습니다. 모델이 아니라 표본을 의심하세요 — '
                  '기준점이 흔들렸거나 케이블이 섞였을 가능성이 큽니다.')
        # 단순 계산과의 차이를 보여줘야 "왜 여러 점이냐"가 화면에서 설명된다.
        if usable_points:
            near = min(usable_points, key=lambda dw: dw[0])
            naive = near[1] * near[0] / size_m
            print(f'  ↳ 가장 가까운 점만으로 계산하면 f_px={naive:.1f} '
                  f'({(naive / fit["f_px"] - 1) * 100:+.1f}% 차이)')
        print('\n적용:')
        print('  ros2 launch robot_arm_perception wrist_camera.launch.py \\')
        print(f'      f_px:={fit["f_px"]:.1f} box_size_m:={size_m}')
        if abs(fit['offset_m']) > 0.01:
            print(f'  ※ 노드가 내는 distance_m 은 **렌즈 기준**입니다. 자로 잰 값과는 '
                  f'약 {fit["offset_m"] * 1000:.0f}mm 차이가 납니다.')

    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({'real_size_m': size_m, 'points': points, 'fit': fit},
                  f, ensure_ascii=False, indent=2)
    print(f'\n원본 표본 저장: {out_path}')
    print('=' * 62)


def main():
    parser = argparse.ArgumentParser(description='손목 캠 거리 곡선 실측')
    parser.add_argument('--size', type=float, default=0.095,
                        help='대상의 **가로로 보이는 변** 실치수(m). 기본 95mm 큐브')
    parser.add_argument('--samples', type=int, default=20,
                        help='거리 한 점당 표본 수(기본 20)')
    parser.add_argument('--out', default='wrist_proximity.json')
    args = parser.parse_args()

    rclpy.init()
    collector = MetricsCollector()
    spin_thread = spin_in_background(collector)

    print('손목 캠 거리 곡선 실측')
    print(f'  대상 가로 실치수 {args.size * 1000:.0f}mm / 점당 {args.samples}표본')
    print('  거리(mm)를 입력하고 Enter. 빈 줄이면 계산하고 끝냅니다.\n')

    deadline = time.time() + 5.0
    while collector.latest() is None and time.time() < deadline and rclpy.ok():
        time.sleep(0.1)
    if collector.latest() is None:
        print(f'⚠️ {collector.topic} 이 조용합니다 — wrist_camera 노드를 먼저 띄우세요.\n')

    points = []
    try:
        while rclpy.ok():
            line = input('거리(mm) > ').strip()
            if not line:
                break
            try:
                distance_m = float(line) / 1000.0
            except ValueError:
                print('  숫자를 mm 로 입력하세요 (예: 250)')
                continue
            if distance_m <= 0.0:
                print('  0보다 커야 합니다')
                continue
            point = _collect_point(collector, distance_m, args.samples)
            _print_point(point)
            if point['bbox_w_median'] is not None:
                points.append(point)
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        if points:
            _report(points, args.size, os.path.abspath(args.out))
        else:
            print('점이 하나도 없어 계산할 게 없습니다.')
        shutdown(collector, spin_thread)


if __name__ == '__main__':
    main()
