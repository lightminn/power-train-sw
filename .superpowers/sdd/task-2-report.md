# Task 2 — Diagnostic snapshot engine report

## Scope implemented

- Added `l515_dashboard/diagnostics.py` with the six exact L515 topic constants.
- Added bounded per-topic arrival deques (default 512 records, 5 s rolling window).
- Added `DiagnosticsTracker.observe(topic, stamp_ns, now_ns)` and immutable
  `DiagnosticsTracker.snapshot(now_ns)` output.
- Snapshot fields cover rolling count/FPS, arrival age, maximum positive header-stamp gap,
  non-increasing header-stamp count, per-topic freshness, and aggregate health.
- Freshness thresholds are explicit: video 0.25 s, CameraInfo 0.50 s, IMU 0.25 s.
- Tracker accepts and stores only topic strings and integer timestamps; snapshots contain only
  frozen scalar records behind a read-only mapping, so no ROS/Image object can be retained.

## TDD evidence

1. Initial required command RED was environment-blocked:
   `python3 -m pytest -q l515_dashboard/tests/test_diagnostics.py`
   → `/usr/bin/python3: No module named pytest`.
2. Project-required conda base equivalent established the feature RED:
   `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_diagnostics.py`
   → collection error, `ModuleNotFoundError: l515_dashboard.diagnostics`.
3. First GREEN after implementation:
   → `6 passed in 0.01s`.
4. Self-review found rolling `count` was cumulative and an unnecessary test-only public property
   had been introduced. The revised rolling-count test failed as expected (`5 != 3`), then the
   implementation was narrowed to `len(arrivals)` and the extra property removed.
5. Post-fix focused GREEN:
   `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_diagnostics.py`
   → `6 passed in 0.01s`.
6. Task 1 + Task 2 regression run:
   `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_config.py l515_dashboard/tests/test_diagnostics.py`
   → `47 passed in 0.04s`.

## Self-review

- Preserved Task 1 `DashboardConfig` and package interfaces without edits.
- Unknown topics fail closed instead of silently allocating unbounded state.
- Equal stamps increment `nonincreasing_count`; only positive stamp deltas contribute to max gap.
- FPS uses arrival time, avoiding corruption from the very header-stamp faults being diagnosed.
- Age clamps at zero if a caller supplies a snapshot time preceding the most recent observation.
- No ROS imports or hardware dependency were introduced.

## Concerns

- The design fixes freshness categories but not numeric thresholds. The selected 0.25/0.50/0.25 s
  values are deliberate 30 Hz operational defaults and may be moved into dashboard configuration
  in a later integration task if runtime tuning is required.
- System `/usr/bin/python3` lacks pytest; tests pass with the documented conda base environment.

## Review fix evidence — rolling timestamp-order state

- Review reproduction test inserted an equal-stamp event, advanced beyond a 1 s window, and then
  inserted a lower stamp as the first event of a new stream window.
- RED command:
  `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_diagnostics.py`
  → `1 failed, 6 passed`; expired snapshot had `count == 0` but
  `nonincreasing_count == 1` at `test_diagnostics.py:53`.
- Root cause: `_TopicState.nonincreasing_count` and `last_stamp_ns` were lifetime scalars independent
  of the bounded arrival deque, so pruning could not expire timestamp-order history.
- Fix: removed both lifetime scalars and derive `nonincreasing_count` exclusively from adjacent
  stamps retained in the bounded rolling deque. No message or image object is retained.
- Focused GREEN after fix:
  `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_diagnostics.py`
  → `7 passed in 0.01s`.
- Final Task 1+2 regression:
  `PATH=/home/light/anaconda3/bin:$PATH python3 -m pytest -q l515_dashboard/tests/test_config.py l515_dashboard/tests/test_diagnostics.py`
  → `48 passed in 0.03s`.
