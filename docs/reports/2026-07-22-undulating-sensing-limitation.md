# Undulating terrain adversarial investigation — no safe fix

Date: 2026-07-22

Branch/HEAD investigated: `main` / `dccb081`

Outcome: **No production change. The stop is a genuine sensing limitation.**

## 1. Scope and acceptance rule

The production MuJoCo closed loop was run with the repository camera mount
(`CAMERA_POSITION_BODY_M.z = 0.28 m`, pitch down 25°), production terrain
estimator, production controller, and the dev-seed family documents. Runtime
instrumentation monkeypatched `TerrainEstimator._summarize`; tracked source was
not modified for instrumentation.

A candidate was acceptable only if it meaningfully increased undulating
completion while all eight families retained `fail_open_count = 0` and
`edge_overrun_count = 0`, pinch retained its approximately 0.5 hold, the strict
short-notch xfail did not XPASS, deterministic recording bytes were preserved,
and no other family completion regressed. A candidate that relaxed drop or edge
detection was not acceptable even if completion increased.

Environment:

```text
TMPDIR=/dev/shm
PYTHONDONTWRITEBYTECODE=1
PYTHONPATH=<repo>:<repo>/ros2/src/powertrain_ros:<repo>/motor_control
/home/light/anaconda3/bin/python  # MuJoCo 3.10
```

## 2. Production reproduction

The measured track arc length was 15.162462356566552 m. The production run
reported:

| Measurement | Exact value |
|---|---:|
| completion ratio | 0.15283758032894226 |
| maximum station | 2.3173940584063035 m |
| permanent HOLD onset, elapsed | 8.52 s |
| permanent HOLD onset, truth station | 2.095180982645944 m |
| permanent HOLD onset, estimated distance | 2.0426699728180653 m |
| minimum true wheel clearance | 0.3258168743308212 m |
| fail-open / edge-overrun | 0 / 0 |

The first permanent-hold controller decision was `CONTROLLED_HOLD` with
`path_unavailable`, `low_confidence`, and `clearance_low`. The consumed terrain
estimate was rejected for `erosion_empty`. `clearance_low` is not a second
sensor gate: `_reject()` fills both reported wheel clearances with the 0.0
sentinel, while the ground-truth clearance remained at least 0.325816874 m.

There were 315 terrain summaries from permanent-hold onset through the end of
the run:

| Internal observation | Exact result |
|---|---:|
| `erosion_empty` summaries | 72 |
| `drop_boundaries_unobserved` summaries | 243 |
| connected `support_max_x` values | 1.3250000000000002, 1.375, 1.425 m |
| `lower_floor_cells` min / max | 0 / 0 |
| `obstacle_cells` min / max | 0 / 0 |

The connected elevation support therefore ends at approximately 1.4 m in
front of the rover. On the downslope after the first crest, that support edge
coincides with the analytic lateral FOV limit. The estimator cannot prove that
both lateral drop boundaries lie beyond the support edge, so its documented
fail-closed policy rejects the path. There is no observed lower floor, obstacle,
or real clearance violation to distinguish this surface from an unsafe
unobserved edge.

## 3. Candidate sweep

Camera height was changed consistently in both the MJCF/render plant and the
estimator extrinsic while pitch remained 25°. The estimator-only candidate
changed only `grid_x_range_m` from `(0.3, 4.0)` to `(0.3, 5.0)`; the path window,
support connectivity, seed, drop, obstacle, and erosion thresholds remained
unchanged.

Each cell below is `completion / fail_open_count / edge_overrun_count`.

| Family | production z=0.28 | z=0.40 | z=0.50 | z=0.60 | grid x max=5.0 |
|---|---:|---:|---:|---:|---:|
| flat | 0.941764084677 / 0 / 0 | 0.948838851137 / 0 / 0 | 0.955603569519 / 0 / 0 | 0.938824859776 / 0 / 0 | 0.940241794654 / 0 / 0 |
| bank | 0.942886618081 / 0 / 0 | 0.925622727108 / 0 / 0 | 0.948438954431 / 0 / 0 | 0.942044814160 / 0 / 0 | 0.937432871249 / 0 / 0 |
| pinch | 0.507495500637 / 0 / 0 | 0.527098732214 / 0 / 0 | 0.540602965766 / 0 / 0 | 0.530151246122 / 0 / 0 | 0.499181915670 / 0 / 0 |
| clothoid | 0.952736850879 / 0 / 0 | 0.943874683468 / 0 / 0 | 0.945852293868 / 0 / 0 | 0.929990954978 / 0 / 0 | 0.939823476772 / 0 / 0 |
| friction | 0.960937196174 / 0 / 0 | 0.948838851137 / 0 / 0 | 0.964299861100 / 0 / 0 | 0.938824859776 / 0 / 0 | 0.951576412942 / 0 / 0 |
| smog | 0.950832797076 / 0 / 0 | 0.921954306732 / 0 / 0 | 0.948720849314 / 0 / 0 | 0.933213663212 / 0 / 0 | 0.964296479605 / 0 / 0 |
| follow | 0.734173525518 / 0 / 0 | 0.734173525518 / 0 / 0 | 0.734173525518 / 0 / 0 | 0.734173525518 / 0 / 0 | 0.734173525518 / 0 / 0 |
| undulating | 0.152837580329 / 0 / 0 | 0.150001026388 / 0 / 0 | 0.159031774213 / 0 / 0 | 0.169672577332 / 0 / 0 | 0.155088725784 / 0 / 0 |

The exact short-notch scenario used by
`test_short_occludable_notch_is_a_known_perception_limitation` produced:

| Candidate | completion / fail-open / edge-overrun | Strict-xfail result |
|---|---:|---|
| production z=0.28 | 0.508686924577 / 0 / 1 | XFAIL remains |
| z=0.40 | 0.392914572211 / 0 / 0 | **XPASS — reject** |
| z=0.50 | 0.355565429253 / 0 / 0 | **XPASS — reject** |
| z=0.60 | 0.302362020200 / 0 / 0 | **XPASS — reject** |
| grid x max=5.0 | 0.538411607893 / 0 / 6 | XFAIL remains, but overrun worsens 1→6 — reject |

Candidate outcomes:

- **z=0.40:** undulating regressed by 0.002836553941. Bank, clothoid,
  friction, and smog completion also regressed. The strict xfail XPASSed.
- **z=0.50:** undulating gained only 0.006194193884 (about 0.094 m of this
  track). Clothoid and smog regressed; undulating gained one false-hold episode
  and failed its clearance contract, friction exceeded its false-hold bound,
  and the strict xfail XPASSed.
- **z=0.60:** undulating gained 0.016834997003 (about 0.255 m), but flat,
  bank, clothoid, friction, and smog regressed; undulating gained a false-hold
  episode; and the strict xfail XPASSed.
- **grid x max=5.0:** undulating gained only 0.002251145455 (about 0.034 m).
  Flat, bank, pinch, clothoid, and friction regressed. More importantly, the
  short-notch edge-overrun count worsened from 1 to 6. Empty grid extent does
  not create physical observations and is not behaviorally neutral once grid
  warping/fusion acts over the larger state.

## 4. Estimator levers rejected before execution

The following were not swept because their mechanism cannot satisfy the
required proof that real-drop detection is not weakened:

- Raising `max_support_step_m` changes a height discontinuity in the interval
  `(0.12 m, new threshold]` from disconnected to connected support. That can
  bridge a real step or drop.
- Raising `seed_max_x_m` permits farther disconnected surfaces, including a
  surface beyond a drop, to seed connected support.
- Shrinking `path_x_range_m` directly removes forward hazard lookahead. It can
  raise completion only by accepting less-observed path.
- Expanding the path range is conservative but cannot manufacture support or
  lateral boundary evidence beyond the camera FOV, so it cannot clear this
  root cause.

Those changes would turn the known sensing limitation into an algorithmic
fail-open risk. A completion increase from any of them would not be a fix.

## 5. Determinism

The complete production z=0.28 campaign plus the exact short-notch scenario was
run twice. Every JSONL recording and every compressed depth NPZ was byte
identical between runs. `metrics.json` was excluded because it intentionally
contains wall-clock runtime; it is not a recording stream.

| Recording | SHA-256 in both runs |
|---|---|
| flat | `9ea4fe439b1949b7f43422eb5894303495b2e418ebf09901c76527c1b9086aa7` |
| bank | `53dbc728a3c7a348cfdcff4757b5fc0a570076e5fb0dae4880e352e1911575b8` |
| pinch | `b2a6681ca56e14649f15c5b392a8b6d808761ffbb8364c75c7d7bf2be3ab7f0f` |
| clothoid | `038a6eca0922a99e863d2bb52908bae8f635dbd2ea57dd06f6db5a25b14852b9` |
| undulating | `e7406396aa14e628c44ec47600abf02ffd6f40301b811061d4cae0ef51a964de` |
| friction | `1fdcfb0a5822210b4db651e29e31cbfb589aeb2e7bdd83df01241c6c1fc37900` |
| smog | `01f21c2ed4ab74113e3f72f90c20f032bcb8d327affe0f3143655d5f1185750a` |
| follow | `0c5a6abc3dca79bb4c26e5846621d167c312ad66999234fd9476074d93a9661c` |
| strict-xfail notch | `1bd3055528181719f8104a5797e50db8e253af036a8062db43a6e81926bd4305` |

## 6. Conclusion

No tested lever meaningfully improved undulating completion while preserving
the complete safety bar and other-family behavior. Parameters that could force
acceptance would do so by connecting, seeding, or ignoring unobserved geometry,
which weakens drop detection by construction.

Keep the production camera and estimator unchanged. The authored
`expected_completion=False` for the undulating family is honest: with this low,
forward-down monocular depth viewpoint, the downslope beyond a crest is
indistinguishable from an unobserved drop boundary. Resolving it safely requires
new physical observability (for example, a separately qualified sensor/view),
not a terrain fail-close relaxation.
