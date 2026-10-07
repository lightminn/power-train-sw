#!/usr/bin/env python3
"""`/arm/calib_status` 파싱 — 진행률을 그리려면 문자열을 뜯어야 한다.

## 형식 (권위는 `teleop_core_node._publish_calib_status`)

    idle
    active,<축 이름>,<lower|upper>,<현재 번호>,<전체 개수>
    done,<적용 수>,<거절 수>
    cancelled

지금 GUI 는 이 문자열을 **그대로 화면에 뿌린다**. `active,arm_joint_2,lower,1,4` 를
운영자가 읽어서 "4축 중 1번째 축의 하한을 재는 중" 으로 해석해야 하는데, 리밋 측정은
손으로 팔을 미는 동안 화면을 흘끗 보는 작업이라 그건 실용적이지 않다.

## 왜 별도 모듈인가

`hw_error_parse.py` 와 같은 선례다 — 문자열 규약은 발행하는 쪽이 조용히 바꿀 수 있고,
그때 깨지는 건 화면이다. 파싱을 한 곳에 모아 pytest 로 고정해 두면 형식이 바뀌었을 때
**테스트가 먼저 깨진다.** ROS 비의존이라 하드웨어 없이 돈다.

⚠️ 모르는 형식은 **버리지 않는다.** `state` 를 `'unknown'` 으로 두고 원문을 그대로
남긴다 — 화면이 "모르는 상태"를 표시할 수 있어야, 형식이 바뀐 사실이 드러난다.
"""

#: 측정 단계 라벨. teleop_core 는 축마다 lower → upper 순서로 진행한다.
STEP_LABELS = {'lower': '하한', 'upper': '상한'}


def parse(raw):
    """`/arm/calib_status` 문자열 → dict.

    항상 `state` 와 `raw` 를 갖고, `active` 면 `joint`·`step`·`step_label`·
    `index`·`total`·`progress`, `done` 이면 `applied`·`rejected` 가 붙는다.
    """
    text = '' if raw is None else str(raw).strip()
    out = {'state': 'unknown', 'raw': text}
    if not text:
        out['state'] = 'idle'
        return out

    parts = [p.strip() for p in text.split(',')]
    head = parts[0]

    if head in ('idle', 'cancelled') and len(parts) == 1:
        out['state'] = head
        return out

    if head == 'active' and len(parts) == 5:
        index, total = _int(parts[3]), _int(parts[4])
        if index is None or total is None or total <= 0:
            return out
        out.update({
            'state': 'active',
            'joint': parts[1],
            'step': parts[2],
            'step_label': STEP_LABELS.get(parts[2], parts[2]),
            'index': index,
            'total': total,
            # 축 하나당 하한·상한 두 단계다 — 축 번호만으로는 진행률이 절반씩 튄다.
            'progress': _progress(index, total, parts[2]),
        })
        return out

    if head == 'done' and len(parts) == 3:
        applied, rejected = _int(parts[1]), _int(parts[2])
        if applied is None or rejected is None:
            return out
        out.update({'state': 'done', 'applied': applied, 'rejected': rejected})
        return out

    return out


def summary(info):
    """화면 한 줄 요약. 파싱 결과를 그대로 받는다."""
    state = info.get('state')
    if state == 'idle':
        return '대기 중'
    if state == 'active':
        return (f"측정 중 — {info['joint']} {info['step_label']} "
                f"({info['index']}/{info['total']}축)")
    if state == 'done':
        rejected = info['rejected']
        text = f"완료 — {info['applied']}축 적용"
        # 거절은 조용히 넘기면 안 된다. teleop_core 가 값이 이상한 축을 버렸다는 뜻이고,
        # 그 축은 **재측정 전까지 리밋이 없는 상태**다.
        return text + (f", {rejected}축 거절(재측정 필요)" if rejected else '')
    if state == 'cancelled':
        return '취소됨 — 기록값 폐기, 토크 복귀'
    return f"알 수 없는 상태: {info.get('raw') or '(빈 값)'}"


def _progress(index, total, step):
    done_steps = (index - 1) * 2 + (1 if step == 'upper' else 0)
    return round(done_steps / (total * 2), 3)


def _int(text):
    try:
        return int(text)
    except (TypeError, ValueError):
        return None
