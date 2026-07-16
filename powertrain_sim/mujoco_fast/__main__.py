"""Command-line entry point for the headless MuJoCo fast runner."""
from __future__ import annotations

import argparse
from collections.abc import Sequence

from ..scenario import SEED_CLASSES, load_scenario
from .runner import run_scenario


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario", help="validated simulator-neutral scenario YAML")
    parser.add_argument("out_dir", help="new or empty run output directory")
    parser.add_argument(
        "--seed-class",
        choices=sorted(SEED_CLASSES),
        help="fail unless scenario.prng.seed_class matches this class",
    )
    arguments = parser.parse_args(argv)
    scenario = load_scenario(arguments.scenario)
    if arguments.seed_class and scenario.prng.seed_class != arguments.seed_class:
        parser.error(
            f"scenario seed class is {scenario.prng.seed_class}, "
            f"not {arguments.seed_class}"
        )
    report = run_scenario(scenario, arguments.out_dir)
    print(report.summary())
    return 0 if report.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
