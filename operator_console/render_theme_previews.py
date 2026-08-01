"""Render exact-size, color-only Mission Control comparison boards.

This is a design-review artifact generator.  It does not import or run any
receiver, telemetry, GStreamer, E-stop, or ops code.
"""
from __future__ import annotations

import argparse
from html import escape
from pathlib import Path

import gi

gi.require_version("GdkPixbuf", "2.0")
from gi.repository import GdkPixbuf  # noqa: E402

from .themes import THEMES


THEME_LABELS = {
    "arctic": "ARCTIC INDUSTRIAL CONTROL",
}


def _svg(theme_name: str, width: int, height: int) -> str:
    t = THEMES[theme_name]
    scale = width / 1920
    header_h = 72 * scale
    tabs_h = 48 * scale
    drawer_h = 28 * scale
    pad = 18 * scale
    gap = 16 * scale
    body_y = header_h + tabs_h
    body_h = height - body_y - drawer_h
    panel_w = 380 * scale
    video_x = pad
    video_y = body_y + pad
    video_w = width - pad * 2 - gap - panel_w
    video_h = body_h - pad * 2
    panel_x = video_x + video_w + gap
    pip_w = video_w * 0.27
    pip_h = pip_w * 9 / 16
    pip_x = video_x + video_w - pip_w - 20 * scale
    pip_y = video_y + video_h - pip_h - 20 * scale
    radius = 16 * scale
    font = "'Pretendard','Noto Sans KR','Noto Sans CJK KR',sans-serif"

    def rect(x, y, w, h, fill, rx=0, stroke="none", sw=0):
        return (
            f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" '
            f'rx="{rx:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw:.1f}"/>'
        )

    def text(x, y, value, size, fill, weight=400, anchor="start"):
        return (
            f'<text x="{x:.1f}" y="{y:.1f}" fill="{fill}" '
            f'font-family="{font}" font-size="{size * scale:.1f}" '
            f'font-weight="{weight}" text-anchor="{anchor}">{escape(value)}</text>'
        )

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        rect(0, 0, width, height, t["app_bg"]),
        rect(0, 0, width, header_h, t["header_bg"]),
        rect(0, header_h, width, tabs_h, t["header_bg"]),
        rect(0, header_h + tabs_h - scale, width, scale, t["border"]),
        text(28 * scale, 30 * scale, "재난 대응 로봇 관제 시스템", 22, t["text_main"], 800),
        text(
            28 * scale, 53 * scale, "실시간 임무 수행 및 안전 상태 확인",
            11, t["text_secondary"], 500,
        ),
    ]

    status_x = 930 * scale
    for index, (label, value, color) in enumerate((
        ("로봇 연결", "연결 대기", t["text_muted"]),
        ("카메라", "연결 대기", t["text_muted"]),
        ("전원", "정보 확인 중", t["warning"]),
        ("안전", "확인 중", t["text_muted"]),
    )):
        x = status_x + index * 170 * scale
        out += [
            f'<circle cx="{x:.1f}" cy="{35 * scale:.1f}" r="{4 * scale:.1f}" fill="{color}"/>',
            text(x + 11 * scale, 32 * scale, label, 10, t["text_muted"], 600),
            text(x + 11 * scale, 48 * scale, value, 11, t["text_main"], 700),
        ]

    estop_x = width - 164 * scale
    out += [
        rect(estop_x, 12 * scale, 144 * scale, 48 * scale, "#C62832", 12 * scale),
        text(estop_x + 72 * scale, 34 * scale, "긴급 정지", 14, "#FFFFFF", 800, "middle"),
        text(estop_x + 72 * scale, 50 * scale, "E-STOP", 9, "#FFFFFF", 700, "middle"),
    ]

    tab_y = header_h + 31 * scale
    for index, label in enumerate(("실시간 화면", "로봇 상태", "관리자 조작")):
        x = 32 * scale + index * 128 * scale
        color = t["primary"] if index == 0 else t["text_secondary"]
        out.append(text(x, tab_y, label, 12, color, 700))
    out.append(rect(25 * scale, header_h + tabs_h - 3 * scale, 92 * scale, 3 * scale, t["primary"], 2 * scale))

    out += [
        rect(video_x, video_y, video_w, video_h, t["video_stage"], radius),
        text(video_x + 22 * scale, video_y + 34 * scale, "전방 카메라", 11, t["text_on_dark"], 700),
        f'<circle cx="{video_x + video_w / 2:.1f}" cy="{video_y + video_h / 2 - 34 * scale:.1f}" '
        f'r="{30 * scale:.1f}" fill="none" stroke="{t["text_muted"]}" stroke-width="{2 * scale:.1f}"/>',
        text(
            video_x + video_w / 2, video_y + video_h / 2 + 25 * scale,
            "전방 시야 준비 중", 20, t["text_on_dark"], 700, "middle",
        ),
        text(
            video_x + video_w / 2, video_y + video_h / 2 + 52 * scale,
            "로봇의 영상 신호를 연결하고 있습니다", 12, t["text_muted"], 400, "middle",
        ),
        rect(pip_x - 3 * scale, pip_y - 3 * scale, pip_w + 6 * scale, pip_h + 6 * scale, t["surface"], radius),
        rect(pip_x, pip_y, pip_w, pip_h, t["video_stage"], radius - 2 * scale),
        text(pip_x + 15 * scale, pip_y + 25 * scale, "작업 카메라", 10, t["text_on_dark"], 700),
        text(pip_x + pip_w / 2, pip_y + pip_h / 2 + 6 * scale, "작업 시야 준비 중", 13, t["text_on_dark"], 700, "middle"),
        text(pip_x + pip_w / 2, pip_y + pip_h / 2 + 28 * scale, "로봇팔 영상 신호를 확인하고 있습니다", 9, t["text_muted"], 400, "middle"),
        rect(panel_x, video_y, panel_w, video_h, t["surface"], radius),
    ]

    px = panel_x + 26 * scale
    py = video_y + 39 * scale
    out += [
        text(px, py, "CURRENT MISSION", 10, t["primary"], 800),
        text(px, py + 46 * scale, "상태 확인 중", 30, t["text_main"], 800),
        text(px, py + 78 * scale, "로봇의 현재 임무 상태를", 12, t["text_secondary"], 400),
        text(px, py + 99 * scale, "확인하고 있습니다", 12, t["text_secondary"], 400),
    ]

    step_y = py + 155 * scale
    step_w = (panel_w - 52 * scale) / 4
    for index, label in enumerate(("탐색", "접근", "도구 작업", "완료")):
        x = px + index * step_w
        color = t["primary"] if index == 0 else t["text_muted"]
        out += [
            text(x + step_w / 2, step_y, label, 10, color, 700, "middle"),
            rect(
                x, step_y + 13 * scale, step_w - 5 * scale, 2 * scale,
                t["primary"] if index == 0 else t["border"], scale,
            ),
        ]

    prep_y = step_y + 66 * scale
    prep_w = panel_w - 52 * scale
    out += [
        rect(px, prep_y, prep_w, 118 * scale, t["primary_soft"], 14 * scale),
        text(px + 18 * scale, prep_y + 37 * scale, "시스템 정보를 준비하고 있습니다", 14, t["text_main"], 700),
        text(px + 18 * scale, prep_y + 69 * scale, "대상이 탐지되면 거리와 작업 도구", 10, t["text_secondary"], 400),
        text(px + 18 * scale, prep_y + 89 * scale, "정보가 표시됩니다", 10, t["text_secondary"], 400),
    ]

    safety_y = prep_y + 142 * scale
    out += [
        rect(px, safety_y, prep_w, 58 * scale, t["surface_soft"], 14 * scale),
        text(px + 17 * scale, safety_y + 35 * scale, "안전 장치", 11, t["text_secondary"], 700),
        text(px + prep_w - 17 * scale, safety_y + 35 * scale, "연결 상태 확인 중", 11, t["text_main"], 700, "end"),
        text(panel_x + panel_w - 18 * scale, video_y + video_h - 18 * scale, THEME_LABELS[theme_name], 8, t["text_muted"], 600, "end"),
        rect(0, height - drawer_h, width, drawer_h, t["header_bg"]),
        rect(0, height - drawer_h, width, scale, t["border"]),
        text(28 * scale, height - 9 * scale, "이벤트 로그  ∧", 9, t["text_muted"], 600),
        "</svg>",
    ]
    return "\n".join(out)


def render(theme_name: str, width: int, height: int, output: Path) -> None:
    svg_path = output.with_suffix(".svg")
    svg_path.write_text(_svg(theme_name, width, height), encoding="utf-8")
    pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_scale(
        str(svg_path), width, height, False,
    )
    pixbuf.savev(str(output), "png", [], [])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for width, height in ((2880, 1800), (1920, 1080), (1600, 900)):
        for theme_name in THEMES:
            render(
                theme_name, width, height,
                args.output_dir / f"theme-{theme_name}-{width}x{height}.png",
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
