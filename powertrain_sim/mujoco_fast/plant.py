"""Thin deterministic wrapper around the MuJoCo fast-mode plant."""
from __future__ import annotations

import math

import mujoco
import numpy as np

from chassis import kinematics
from chassis.kinematics import ChassisGeometry, SolveResult, default_geometry

from ..scenario import Scenario
from .model_builder import build_mjcf


INITIAL_SETTLE_S = 0.5


def _id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    identifier = int(mujoco.mj_name2id(model, object_type, name))
    if identifier < 0:
        raise ValueError(f"MuJoCo object not found: {name}")
    return identifier


def _quaternion_roll_yaw(quaternion: np.ndarray) -> tuple[float, float]:
    w, x, y, z = (float(value) for value in quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, yaw


class MujocoFastPlant:
    """Own one headless MuJoCo model and expose value-based plant state."""

    def __init__(
        self,
        scenario: Scenario,
        *,
        geometry: ChassisGeometry | None = None,
        build_kwargs: dict | None = None,
    ) -> None:
        self.scenario = scenario
        self.geometry = geometry or default_geometry()
        self.xml = build_mjcf(
            scenario,
            geometry=self.geometry,
            **(build_kwargs or {}),
        )
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.physics_dt_s = float(self.model.opt.timestep)
        self.substeps_per_clock_step = int(
            round(scenario.clock.dt_s / self.physics_dt_s)
        )
        if self.physics_dt_s > 0.005 + 1e-12 or not math.isclose(
            self.substeps_per_clock_step * self.physics_dt_s,
            scenario.clock.dt_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("MuJoCo physics timestep must exactly divide scenario clock dt")

        self.root_free_joint_id = _id(
            self.model,
            mujoco.mjtObj.mjOBJ_JOINT,
            "root_free",
        )
        self._root_dof_address = int(
            self.model.jnt_dofadr[self.root_free_joint_id]
        )
        self._actuator_ids = {
            name: _id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in (
                *(f"steer_{wheel.name}" for wheel in self.geometry.wheels if wheel.steerable),
                *(f"drive_{wheel.name}" for wheel in self.geometry.wheels),
            )
        }
        self._drive_joint_ids = {
            wheel.name: _id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                f"drive_joint_{wheel.name}",
            )
            for wheel in self.geometry.wheels
        }
        self._steer_joint_ids = {
            wheel.name: _id(
                self.model,
                mujoco.mjtObj.mjOBJ_JOINT,
                f"steer_joint_{wheel.name}",
            )
            for wheel in self.geometry.wheels
            if wheel.steerable
        }
        self._wheel_body_ids = {
            wheel.name: _id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                f"wheel_{wheel.name}",
            )
            for wheel in self.geometry.wheels
        }
        self._wheel_geom_ids = {
            wheel.name: _id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                f"wheel_geom_{wheel.name}",
            )
            for wheel in self.geometry.wheels
        }
        self.base_body_id = _id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "base_link",
        )
        self.depth_site_id = _id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            "depth_site",
        )
        self._last_solve_result: SolveResult | None = None
        mujoco.mj_forward(self.model, self.data)
        settle_steps = int(math.ceil(INITIAL_SETTLE_S / self.physics_dt_s))
        for _ in range(settle_steps):
            mujoco.mj_step(self.model, self.data)
        # Settling is an initialization detail, outside the scenario clock. Start
        # the measured scenario from rest rather than carrying residual suspension
        # velocity out of the pre-roll.
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.data.time = 0.0

    @property
    def last_solve_result(self) -> SolveResult | None:
        return self._last_solve_result

    def apply_command(self, v_m_s: float, omega_rad_s: float) -> SolveResult:
        """Apply the production kinematics result without a simulator Ackermann path."""
        result = kinematics.solve(self.geometry, v_m_s, omega_rad_s)
        for wheel in self.geometry.wheels:
            command = result.wheels[wheel.name]
            if wheel.steerable:
                self.data.ctrl[self._actuator_ids[f"steer_{wheel.name}"]] = math.radians(
                    command.steer_deg
                )
            self.data.ctrl[self._actuator_ids[f"drive_{wheel.name}"]] = (
                command.drive_turns_per_s * 2.0 * math.pi
            )
        self._last_solve_result = result
        return result

    def actuator_controls(self) -> dict[str, float]:
        return {
            name: float(self.data.ctrl[identifier])
            for name, identifier in self._actuator_ids.items()
        }

    def step_clock_interval(self) -> None:
        for _ in range(self.substeps_per_clock_step):
            mujoco.mj_step(self.model, self.data)

    def measured_wheel_turns_per_s(self) -> dict[str, float]:
        measured = {}
        for wheel in self.geometry.wheels:
            joint_id = self._drive_joint_ids[wheel.name]
            dof_address = int(self.model.jnt_dofadr[joint_id])
            measured[wheel.name] = float(self.data.qvel[dof_address]) / (2.0 * math.pi)
        return measured

    def steering_angles_deg(self) -> dict[str, float]:
        angles = {}
        for wheel in self.geometry.wheels:
            joint_id = self._steer_joint_ids.get(wheel.name)
            if joint_id is None:
                angles[wheel.name] = 0.0
            else:
                qpos_address = int(self.model.jnt_qposadr[joint_id])
                angles[wheel.name] = math.degrees(float(self.data.qpos[qpos_address]))
        return angles

    def imu_raw(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        gyro = tuple(float(value) for value in self.data.sensor("imu_gyro").data)
        acceleration = tuple(float(value) for value in self.data.sensor("imu_accel").data)
        return gyro, acceleration

    def ground_truth_pose(self) -> tuple[np.ndarray, float, float, float, float]:
        position = np.array(self.data.sensor("base_framepos").data, dtype=float, copy=True)
        quaternion = np.array(
            self.data.sensor("base_framequat").data,
            dtype=float,
            copy=True,
        )
        bank_rad, yaw_rad = _quaternion_roll_yaw(quaternion)
        body_matrix = np.asarray(self.data.xmat[self.base_body_id], dtype=float).reshape(3, 3)
        linear_world = np.asarray(
            self.data.qvel[self._root_dof_address : self._root_dof_address + 3],
            dtype=float,
        )
        linear_speed = float(np.dot(linear_world, body_matrix[:, 0]))
        gyro, _ = self.imu_raw()
        return position, yaw_rad, bank_rad, linear_speed, float(gyro[2])

    def wheel_contact_points_world(self) -> dict[str, tuple[float, float, float]]:
        contacts: dict[str, list[np.ndarray]] = {wheel.name: [] for wheel in self.geometry.wheels}
        geom_to_wheel = {identifier: name for name, identifier in self._wheel_geom_ids.items()}
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            wheel_name = geom_to_wheel.get(int(contact.geom1)) or geom_to_wheel.get(
                int(contact.geom2)
            )
            if wheel_name is not None:
                contacts[wheel_name].append(np.array(contact.pos, dtype=float, copy=True))
        output = {}
        for wheel in self.geometry.wheels:
            points = contacts[wheel.name]
            if points:
                point = np.mean(points, axis=0)
            else:
                point = np.array(
                    self.data.xpos[self._wheel_body_ids[wheel.name]],
                    dtype=float,
                    copy=True,
                )
                point[2] -= self.geometry.wheel_radius_m
            output[wheel.name] = tuple(float(value) for value in point)
        return output


__all__ = ("INITIAL_SETTLE_S", "MujocoFastPlant")
