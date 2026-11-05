# Bundled UI fonts

- Manrope variable: `https://github.com/google/fonts/tree/main/ofl/manrope`
- SUIT variable: `https://github.com/sun-typeface/SUIT/tree/main/fonts/variable/ttf`

Both are distributed under the SIL Open Font License 1.1. The unmodified
upstream licenses accompany the font files. Fontconfig registers these files
for the console process only; no desktop font installation or preference
change is performed. The UI uses Manrope for Latin/numerals, with SUIT for
Hangul. No runtime network access is needed.

Default console font (2026-10-16): **눈누 기초고딕 Regular**.
Source: https://noonnu.cc/font_page/1496
See `NoonnuBasicGothic-LICENSE.txt`. The native TTF is a container conversion of
the same WOFF2 used by the local 3D view; outlines and font names are unchanged.
Registration applies to this process, with Noto Sans CJK KR fallback.
