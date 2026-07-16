# Environment regression fixtures

`manifest.yaml` pins simulator-neutral environment regressions to sensor
contract version 1. `powertrain_sim/scenario.py` remains the executable source
of truth; `scenario.schema.yaml` documents the same keys for reviewers and
tests and is not a second parser.

## Adding or changing an entry

1. Add a parser-valid repository-relative scenario, or use the exact
   `procedural:<seed_class>:<seed>` reference. Never inspect or tune against a
   `hidden_evaluation` seed.
2. Run `powertrain_sim.scenario.load_scenario()` for a file scenario. Use only
   SI values, the declared frames, `PCG64`, and explicit track, sensor, fault,
   and expected-metric fields.
3. Choose one Task 7 fixture class and one implemented backend. A nominal
   scenario may be a negative-control entry for a class; the injected fault
   still has to be explicit in the scenario before an expected reject reason
   may be claimed.
4. For a file scenario, checksum the exact repository bytes:

   ```bash
   sha256sum powertrain_sim/scenarios/example.yaml
   ```

   Whitespace and comments are part of this checksum. Any byte change requires
   review and a manifest checksum update.
5. For a procedural reference, regenerate with the default frozen parameters
   and hash canonical JSON, not YAML text:

   ```python
   from powertrain_sim.procedural import (
       GenerationParameters,
       canonical_json_sha256,
       generate_scenario,
   )

   document = generate_scenario(
       GenerationParameters(), seed=7, seed_class="dev"
   )
   print(canonical_json_sha256(document))
   ```

6. Keep the same `id` across backends only when they represent the same
   fixture. The `(id, source)` pair must be unique and every copy of an id must
   use the same tolerance map. Backend comparison uses only metrics that both
   executions actually report; unavailable optional backends are written as
   `SKIPPED` rather than silently passing.
7. Run the manifest and inspect both per-entry results and comparisons:

   ```bash
   PYTHONPATH=ros2/src/powertrain_ros:motor_control:. python \
     scripts/run_autonomy_regression.py \
     --manifest tests/fixtures/environment/manifest.yaml \
     --out /tmp/regression_out.json
   ```

