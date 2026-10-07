#!/usr/bin/env python3
"""손목 캠 파지 판정 밴드 실측 — "제대로 물었다"의 수치 범위를 정한다 (2026-08-13 신설).

## 이 스크립트가 답하는 질문

FSM 2·3단계는 "그리퍼가 상자를 물고 있나"를 손목 캠으로 확인하려 한다. 그러려면 먼저
**정상 파지 상태의 지표가 실패 상태와 실제로 갈리는지**를 알아야 한다. 안 갈리면
임계값을 정할 게 아니라 관측 조건(ROI·조명·카메라 각도)을 바꿔야 한다 — 그 판단을
사람에게 넘기는 게 이 스크립트의 목적이고, 그래서 "밴드"만이 아니라 **분리 여부**를
같이 낸다.

## 어떤 지표를 보는가

`fill`(마스크 픽셀 / 화면 픽셀)과 중심 `u`/`v` 셋뿐이다. **bbox 폭·높이와 주축각은
쓰지 않는다** — `arm_joint_5`(손목 롤) 축이 카메라 시선과 거의 나란해서 물린 상자가
이미지 평면 안에서 제자리 회전을 하고, 정사각형이 회전하면 축정렬 bbox 면적이 √2배까지
변한다. `fill` 과 `centroid` 만 그 회전에 불변이다(`wrist_metrics` 모듈 docstring).

## 쓰는 법

    ros2 launch robot_arm_perception wrist_camera.launch.py     # 다른 터미널
    source /root/ros2_ws/install/setup.bash
    python3 src/robot_arm_perception/scripts/measure_wrist_grasp_band.py

명령: `g`=정상 파지 / `e`=빈 그리퍼 / `s`=어긋난 파지(반쯤 물림·미끄러짐) /
`r`=중간 결과 / 빈 줄=계산하고 종료.

각 조건마다 **손목 롤을 조금씩 돌려 가며** 여러 번 재는 게 좋다(같은 자세만 재면
회전 불변성을 검증하지 못한 밴드가 나온다). 3조건 각 3회 이상을 권장한다.

⚠️ 실패 조건(`e`/`s`)을 하나도 안 재면 분리 여부를 판정할 근거가 없다 — 그 경우
`separated=False` 로 보고하고 임계값을 확정하지 않는다.
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

CONDITIONS = {
    'g': ('grasp', '정상 파지'),
    'e': ('empty', '빈 그리퍼'),
    's': ('slipped', '어긋난 파지'),
}


def _extract(samples):
    """표본 묶음 → 지표별 리스트.

    ⚠️ 파지 거리에서는 상자가 화면을 가득 채워 네 변에 닿는 게 **정상**이라 잘림을
    버리지 않는다(거리 실측과 반대다 — 거기서는 잘리면 겉보기 크기가 거짓말한다).
    가림(`occluded`)만 버린다: 핑거가 상자를 덮은 프레임은 파지 성공/실패 어느 쪽의
    대표값도 아니다.
    """
    def ok(sample):
        # max_trim_ratio=1.0 = 잔가지 필터를 끈다. 파지 밴드는 **운용 중 분포**를 재는
        # 것이라, 운용에서 그대로 쓸 값(잘라낸 뒤의 값)을 그대로 표본에 넣어야 한다.
        return usable(sample, allow_clipped=True, max_trim_ratio=1.0)

    detected = [s for s in samples if ok(s)]
    return {
        'n': len(samples),
        'n_detected': len(detected),
        # 미검출 프레임의 fill 은 0 이다 — 이건 결측이 아니라 "아무것도 안 보인다"는 관측이다.
        'fill': [float(s['fill']) if ok(s) else 0.0 for s in samples],
        'u': [float(s['u']) for s in detected],
        'v': [float(s['v']) for s in detected],
    }


def _merge(chunks):
    out = {'n': 0, 'n_detected': 0, 'fill': [], 'u': [], 'v': []}
    for chunk in chunks:
        out['n'] += chunk['n']
        out['n_detected'] += chunk['n_detected']
        for key in ('fill', 'u', 'v'):
            out[key].extend(chunk[key])
    return out


def _summary_line(label, data):
    if not data['n']:
        return f'  {label:<10} 표본 없음'
    rate = data['n_detected'] / data['n'] * 100.0
    fill_med, fill_sigma = wrist_metrics.robust_stats(data['fill'])
    return (f'  {label:<10} n={data["n"]:<4} 검출률={rate:5.1f}%  '
            f'fill={fill_med:.4f}±{fill_sigma:.4f}')


def _report(store, k, out_path):
    merged = {name: _merge(chunks) for name, chunks in store.items()}
    print('\n' + '=' * 66)
    for key in ('grasp', 'empty', 'slipped'):
        label = next(v[1] for v in CONDITIONS.values() if v[0] == key)
        print(_summary_line(label, merged.get(key, {'n': 0})))

    grasp = merged.get('grasp')
    if not grasp or not grasp['n']:
        print('\n정상 파지 표본이 없어 밴드를 낼 수 없습니다.')
        return

    others = _merge([merged[k2] for k2 in ('empty', 'slipped') if k2 in merged])
    print('-' * 66)
    bands = {}
    for metric in ('fill', 'u', 'v'):
        # u/v 는 실패 조건에서 표본이 거의 없다(빈 그리퍼는 애초에 검출이 안 된다).
        # 그래도 밴드는 내야 한다 — 정상 파지의 자리 이탈을 잡는 게 그 지표의 역할이다.
        band = wrist_metrics.suggest_band(grasp[metric], others[metric], k=k)
        bands[metric] = band
        if band['center'] is None:
            print(f'  {metric:<5} 표본 없음')
            continue
        verdict = ('갈림' if band['separated']
                   else ('겹침!' if band['n_other'] else '비교 표본 없음'))
        print(f'  {metric:<5} 중앙 {band["center"]:+.4f}  '
              f'밴드 [{band["lo"]:+.4f}, {band["hi"]:+.4f}]  '
              f'(±{k}σ, σ={band["sigma"]:.4f})  {verdict}')
        if band['n_other'] and not band['separated']:
            print(f'        ⚠️ 실패 상태 표본 {band["intruders"]}/{band["n_other"]}개가 '
                  '밴드 안에 들어옵니다.')

    fill_band = bands.get('fill', {})
    print('-' * 66)
    if fill_band.get('separated'):
        print('판정: 정상 파지가 실패 상태와 **갈립니다** — FSM 2단계로 넘어가도 됩니다.')
        print(f'  여유(margin) = {fill_band["margin"]:.4f} fill')
        print('\n적용값(파지 확인 게이트):')
        print(f'  wrist_fill_min: {max(0.0, fill_band["lo"]):.4f}')
        print(f'  wrist_fill_max: {min(1.0, fill_band["hi"]):.4f}')
        for metric in ('u', 'v'):
            band = bands.get(metric, {})
            if band.get('center') is not None:
                print(f'  wrist_{metric}_min: {band["lo"]:+.4f}   '
                      f'wrist_{metric}_max: {band["hi"]:+.4f}')
    elif fill_band.get('n_other'):
        print('판정: **갈리지 않습니다.** 임계값을 정하지 마세요 — 이 상태로 FSM 에 물리면')
        print('      실패 파지를 성공으로 보고합니다. ROI 를 그리퍼 쪽으로 더 좁히거나,')
        print('      조명을 고정하거나, 카메라 각도를 바꾼 뒤 다시 재는 게 순서입니다.')
    else:
        print('판정: 실패 상태(빈 그리퍼/어긋난 파지) 표본이 없어 분리를 확인할 수 없습니다.')
        print('      `e`/`s` 로 최소 한 번씩은 재야 이 실측이 의미가 있습니다.')

    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump({'k_sigma': k, 'conditions': merged, 'bands': bands},
                  f, ensure_ascii=False, indent=2)
    print(f'\n원본 표본 저장: {out_path}')
    print('=' * 66)


def main():
    parser = argparse.ArgumentParser(description='손목 캠 파지 밴드 실측')
    parser.add_argument('--samples', type=int, default=30, help='1회 수집 표본 수(기본 30)')
    parser.add_argument('--k', type=float, default=3.0, help='밴드 폭 = 중앙값 ± k σ')
    parser.add_argument('--out', default='wrist_grasp_band.json')
    args = parser.parse_args()

    rclpy.init()
    collector = MetricsCollector()
    spin_thread = spin_in_background(collector)

    print('손목 캠 파지 밴드 실측')
    print('  g=정상 파지  e=빈 그리퍼  s=어긋난 파지  r=중간 결과  빈 줄=종료')
    print(f'  1회 {args.samples}표본 / 밴드 ±{args.k}σ\n')

    deadline = time.time() + 5.0
    while collector.latest() is None and time.time() < deadline and rclpy.ok():
        time.sleep(0.1)
    if collector.latest() is None:
        print(f'⚠️ {collector.topic} 이 조용합니다 — wrist_camera 노드를 먼저 띄우세요.\n')

    store = {}
    try:
        while rclpy.ok():
            key = input('[g/e/s/r] > ').strip().lower()
            if not key:
                break
            if key == 'r':
                _report(store, args.k, os.path.abspath(args.out))
                continue
            if key not in CONDITIONS:
                print('  g / e / s / r 중 하나를 입력하세요')
                continue
            name, label = CONDITIONS[key]
            print(f'  [{label}] 자세를 만든 뒤 수집합니다… ({args.samples}표본)')
            samples, timed_out = collector.collect(args.samples)
            if timed_out:
                print(f'  ⚠️ 타임아웃 — {len(samples)}개만 받았습니다.')
            if not samples:
                continue
            chunk = _extract(samples)
            store.setdefault(name, []).append(chunk)
            print(_summary_line(label, chunk))
    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        if store:
            _report(store, args.k, os.path.abspath(args.out))
        else:
            print('표본이 없어 계산할 게 없습니다.')
        shutdown(collector, spin_thread)


if __name__ == '__main__':
    main()
