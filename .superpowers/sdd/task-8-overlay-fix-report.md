# Task 8 OverlayFS path identity fix report

## Root cause

`ResourceGuard` and `UnixControlServer` recorded only `(st_dev, st_ino)` for owned
lock/socket paths. Jetson's OverlayFS can immediately reuse both values after an
unlink, so cleanup could mistake a successor path for the path it created and
delete it.

## TDD evidence

Before the production change, the three focused regressions failed:

- legacy two-field metadata deleted an unverifiable successor;
- `claim_socket()` persisted only two identity fields;
- simulated OverlayFS reuse caused server stop to delete a successor with the
  same device/inode but a different ctime/type.

Command:

`PYTHONPATH=ros2/src/powertrain_ros /home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_resource_guard.py::test_legacy_two_field_socket_identity_is_not_safe_to_unlink l515_dashboard/tests/test_resource_guard.py::test_claim_persists_full_filesystem_identity l515_dashboard/tests/test_control_server.py::test_stop_preserves_overlayfs_successor_with_reused_inode`

Result before fix: `3 failed`.

## Fix

- Added one no-follow filesystem identity implementation shared by the guard and
  control server: `(st_dev, st_ino, st_ctime_ns, S_IFMT(st_mode))`.
- Persisted the four fields in new lock metadata.
- Treat legacy/malformed identity metadata as unverifiable and preserve the
  socket path. The stale lock itself can still be reclaimed safely.
- Applied the same comparison to guard claim/release/stale reclaim and server
  bind rollback/stop.
- Captured a published lock identity only after removing its temporary hard
  link, because that unlink changes the surviving inode's ctime.

## Verification

- Focused resource/control tests: `23 passed in 0.91s`.
- Full dashboard suite: `197 passed in 4.71s`.
- `/home/light/anaconda3/bin/python -m compileall -q l515_dashboard`: exit 0.
- `git diff --check`: exit 0.

No Jetson state or non-code project documentation was changed.

## Canonical-path TOCTOU follow-up

A review found that even four-field `lstat` comparison followed by `unlink`
left a canonical-path replacement window. Cleanup now uses a shared quarantine
primitive:

1. Open canonical with `O_PATH|O_NOFOLLOW` and verify the recorded identity.
2. Atomically rename canonical to an unpredictable same-directory quarantine.
3. Verify the quarantined entry is the same open inode, accounting for rename's
   ctime update through the open descriptor.
4. Unlink only that verified quarantine.
5. Restore an unknown quarantine with Linux `renameat2(RENAME_NOREPLACE)`.
   If canonical was recreated, preserve both paths and raise
   `PathOwnershipConflict`; never delete the unknown quarantine.

Deterministic tests replace canonical exactly during cleanup and cover both an
owned quarantine with a canonical successor and an unknown quarantine whose
canonical path is recreated. The primitive is used for ControlServer
rollback/stop and ResourceGuard socket/lock stale cleanup, claim, and release.

Fresh follow-up verification:

- focused quarantine and integrations: `5 passed in 0.13s`;
- full dashboard suite: `202 passed in 5.40s`;
- compileall and `git diff --check`: exit 0.
