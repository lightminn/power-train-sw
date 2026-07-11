# Task 8 definitive endpoint and ownership report

## Root cause and architecture decision

Conditional pathname unlink cannot be made race-free against canonical-path
replacement. Device/inode/ctime checks and quarantine reduced individual races
but retained filesystem socket cleanup as unnecessary complexity.

The definitive design removes filesystem control-socket paths and lock-file
deletion entirely.

## Implementation

- Gateway and Dashboard share `abstract_address()` to convert the display form
  `@powertrain-l515-gateway` to Linux's leading-NUL abstract Unix address.
- `UnixControlServer` binds only the abstract address. It creates, chmods,
  renames, or unlinks no socket pathname.
- The server reads Linux `SO_PEERCRED` and accepts commands only from the same
  effective UID. Tests can inject an authorizer to exercise rejection before a
  handler sees the request.
- `ResourceGuard` opens persistent `/run/powertrain/l515-gateway.lock` with
  `O_CREAT|O_NOFOLLOW|O_CLOEXEC`, verifies an owner-controlled regular file,
  and acquires `flock(LOCK_EX|LOCK_NB)`.
- While locked, current PID and `/proc/<pid>/stat` start identity are written,
  truncated, and fsynced. Old file contents are ordinary stale metadata.
- Release unlocks and closes the descriptor but never unlinks the lock file.
- The former path identity, quarantine removal, socket claim, mutex, stale-path
  reclaim, and PID-based ownership decisions were removed.

## Tests

Coverage includes:

- two simultaneous lock contenders;
- release/reacquire while the persistent file remains;
- stale metadata overwrite under lock;
- symlink rejection without target modification;
- explicit proof that guard release and abstract server stop call no unlink;
- same-UID default peer authorization and injected unauthorized rejection
  before command handling;
- shared abstract endpoint conversion for client and server;
- real Dashboard/fake Gateway process interaction over an abstract endpoint.

No Jetson/HIL operation was performed for this architecture-only correction.

Final verification:

- bounded control-server suite: `8 passed in 0.83s`;
- full Dashboard suite under a 30-second external bound: `188 passed in 4.77s`;
- `compileall` and `git diff --check`: exit 0;
- no pytest process remained after either run.

## Container namespace closure

The Jetson `powertrain_ros` service now bind-mounts host `/run/powertrain` onto
the same container path. Replacement containers therefore contend on the same
persistent flock inode rather than separate container overlay files. Host
networking already shares the abstract Unix namespace.

Gateway startup now orders ownership barriers before hardware:

`ResourceGuard → abstract ControlServer → SDK source → ROS → optional SRT`

A deterministic duplicate abstract bind test proves that `source.start()` is
never called when `EADDRINUSE` occurs, while normal cleanup rolls back the
server and guard. Compose coverage asserts the exact shared mount.

Final namespace-fix verification: compose config exit 0, `git diff --check`
exit 0, compileall exit 0, and the full Dashboard suite completed within its
30-second bound with `191 passed in 4.94s` and no remaining pytest process.
