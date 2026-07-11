# Task 3 — Video modes and latest-frame handoff report

## Scope implemented

- Added `FrameMode.COLOR`, `DEPTH`, and `SIDE_BY_SIDE`.
- Added `render_frame(mode, color, depth, width, height)` with contiguous uint8
  BGR output at 640×480 for single modes and 1280×480 for side-by-side.
- Color is passed through at the configured size.
- Depth uses a deterministic fixed 0–5000 mm mapping to 0–255 followed by
  OpenCV TURBO; zero/invalid depth remains black and values above 5000 mm clip.
- Added thread-safe `LatestVideoFrames` color/depth one-slot overwrite and
  consume semantics. Inputs are copied on put, rendering occurs outside the
  lock, and every take clears both slots so a later mode switch cannot replay
  an unselected old frame.
- Missing required inputs return `None`; no black placeholder or prior frame is
  synthesized.
- No streamer, supervisor, ROS bridge, or TUI behavior was added.

## TDD evidence

1. Initial feature RED:
   `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_frame_modes.py`
   → collection failed with `ModuleNotFoundError: l515_dashboard.frame_modes`.
2. Initial GREEN after the minimal implementation:
   same focused command → `12 passed in 0.17s`.
3. Self-review identified a mode-switch stale replay case: taking COLOR left an
   older DEPTH slot available. A new test reproduced it RED:
   focused command → `1 failed, 12 passed`; the subsequent DEPTH take returned
   the old mapped frame instead of `None`.
4. Fix: each atomic take now snapshots and clears both slots. Focused and
   Tasks 1–3 regression verification are recorded below after final cleanup.

## Final verification evidence

- Focused:
  `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_frame_modes.py`
  → `13 passed in 0.18s`.
- Tasks 1–3 regression:
  `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_config.py l515_dashboard/tests/test_diagnostics.py l515_dashboard/tests/test_frame_modes.py`
  → `61 passed in 0.25s`.
- Flake8 on both Task 3 files, package compileall, and `git diff --check` all
  exited 0 with no output.

## Self-review

- Latest overwrite is bounded to one ndarray per input and protected by one
  lock; no queue can accumulate.
- The lock covers only snapshot/slot mutation, not OpenCV rendering.
- Copy-on-put prevents a producer from mutating a frame while a consumer uses
  it.
- Incomplete side-by-side attempts consume the unpaired input, preventing it
  from being combined later with a fresh frame.
- The depth scale is frame-independent, so identical millimeter values always
  produce identical BGR values; zeros are explicitly restored to black after
  applying TURBO.
- Reviewed scope against the Task 3 brief and approved design; only the two
  requested files were added.

## Concerns

- The approved design requires a fixed visible depth range but does not name
  its endpoint. Task 3 fixes it at 5000 mm as an explicit deterministic
  dashboard visualization constant. If operations later approve another fixed
  range, the constant and pixel-contract test must change together.
- Host `/usr/bin/python3` does not provide pytest; all executable evidence uses
  the documented conda base interpreter through `PATH`.
