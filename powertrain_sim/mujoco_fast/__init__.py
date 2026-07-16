"""Headless MuJoCo fast-mode bridge for WP6-S P0.

The third-party :mod:`mujoco` package is imported only by modules inside this
subpackage, so the simulator-neutral part-one contracts remain importable on
ROS hosts without MuJoCo installed.
"""

__all__: tuple[str, ...] = ()
