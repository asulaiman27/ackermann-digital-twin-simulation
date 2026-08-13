"""Surface-dependent Ackermann dynamics and online friction identification.

The legacy Stage-1 simulator intentionally remains a kinematic bicycle.  This
module is the experiment-only dynamic plant used by the environment-indexed pH
study.  Friction enters only through tire-force saturation, so trajectories are
independent of ``mu`` while all requested forces remain inside the grip circle.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import math
from statistics import NormalDist
from typing import Any, Iterable

import numpy as np


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


@dataclass(frozen=True)
class DynamicBicycleConfig:
    mass: float = 3.50
    yaw_inertia: float = 0.055
    wheelbase: float = 0.324
    front_length: float = 0.162
    rear_length: float = 0.162
    front_cornering_stiffness: float = 24.0
    rear_cornering_stiffness: float = 26.0
    gravity: float = 9.81
    dt: float = 0.10
    max_accel: float = 1.50
    max_brake: float = 2.50
    max_steer_rate: float = 2.0
    max_steering_angle: float = 0.396
    v_max: float = 0.80
    v_reverse_max: float = 0.25
    speed_time_constant: float = 0.25
    # Aerodynamics.  ``aero_drag`` is the longitudinal drag coefficient Cd;
    # frontal and side areas are derived from the active vehicle dimensions.
    aero_drag: float = 0.32
    aero_side_drag: float = 1.00
    air_density: float = 1.225
    wind_speed: float = 0.0
    # Direction the wind travels toward in world coordinates: 0 degrees is +x,
    # 90 degrees is +y.  This is intentionally not the meteorological
    # "coming from" convention, so the force direction is unambiguous.
    wind_direction_deg: float = 0.0
    rolling_resistance: float = 0.10
    low_speed_regularizer: float = 0.15
    integration_substeps: int = 4
    kinematic_blend_speed: float = 0.35
    # Renderable geometry.  These do not change the bicycle equations, but
    # keep the visual model and the physical model in the same preset.
    track_width: float = 0.22
    body_length: float = 0.62
    body_width: float = 0.28
    body_height: float = 0.14
    wheel_radius: float = 0.055
    wheel_inertia: float = 0.002
    tire_longitudinal_stiffness: float = 30.0
    suspension_stiffness: float = 120.0
    suspension_damping: float = 3.0
    com_height: float = 0.09
    roll_inertia: float = 0.070
    pitch_inertia: float = 0.080
    max_drive_torque: float = 1.20


def aerodynamic_force_body(
    velocity_world: np.ndarray,
    rotation_world_from_body: np.ndarray,
    cfg: DynamicBicycleConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return aerodynamic force, application point, and relative air velocity.

    The force uses relative air velocity and separate longitudinal/side-force
    areas.  This produces headwind, tailwind, and crosswind effects while
    scaling with the active vehicle dimensions.
    """
    wind_direction = math.radians(float(cfg.wind_direction_deg))
    wind_world = np.asarray([
        cfg.wind_speed * math.cos(wind_direction),
        cfg.wind_speed * math.sin(wind_direction),
        0.0,
    ], dtype=np.float64)
    relative_air_world = np.asarray(velocity_world, dtype=np.float64) - wind_world
    relative_air_body = rotation_world_from_body.T @ relative_air_world
    frontal_area = max(0.005, float(cfg.body_width) * float(cfg.body_height) * 0.85)
    side_area = max(0.008, float(cfg.body_length) * float(cfg.body_height) * 0.85)
    dynamic_pressure = 0.5 * max(float(cfg.air_density), 0.0)
    force_body = np.asarray([
        -dynamic_pressure * max(float(cfg.aero_drag), 0.0) * frontal_area
        * relative_air_body[0] * abs(relative_air_body[0]),
        -dynamic_pressure * max(float(cfg.aero_side_drag), 0.0) * side_area
        * relative_air_body[1] * abs(relative_air_body[1]),
        0.0,
    ], dtype=np.float64)
    # A small forward center-of-pressure offset makes a crosswind produce a
    # visible yaw moment instead of acting as a force through the COM.
    application_point = np.asarray([0.08 * float(cfg.body_length), 0.0, 0.0], dtype=np.float64)
    return force_body, application_point, relative_air_body


@dataclass
class DynamicBicycleState:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    steering: float = 0.0

    @property
    def pose(self) -> np.ndarray:
        return np.asarray([self.x, self.y, self.heading], dtype=np.float64)

    def copy(self) -> "DynamicBicycleState":
        return DynamicBicycleState(**asdict(self))


@dataclass(frozen=True)
class DynamicCommand:
    speed: float
    steering: float


@dataclass
class DynamicStep:
    state: DynamicBicycleState
    lateral_acceleration: float
    longitudinal_force: float
    front_lateral_force: float
    rear_lateral_force: float
    supplied_work: float
    dissipated_energy: float
    kinetic_energy: float
    saturated: bool
    friction_utilization: float
    # Body-frame force/moment decomposition for inspection and digital-twin
    # rendering.  The existing scalar fields above remain the stable API used
    # by the experiment code.
    force_breakdown: dict[str, dict[str, Any]] = field(default_factory=dict)


def steering_curvature_limit(cfg: DynamicBicycleConfig) -> float:
    return math.tan(cfg.max_steering_angle) / cfg.wheelbase


def grip_curvature_limit(speed: float, mu: float, cfg: DynamicBicycleConfig) -> float:
    if abs(speed) < 1e-6:
        return steering_curvature_limit(cfg)
    return min(steering_curvature_limit(cfg), max(float(mu), 1e-6) * cfg.gravity / (speed * speed))


def braking_distance(speed: float, mu: float, cfg: DynamicBicycleConfig) -> float:
    return speed * speed / (2.0 * max(float(mu), 1e-6) * cfg.gravity)


def _combined_grip_lateral(raw_fy: float, normal: float, fx: float, mu: float) -> tuple[float, bool, float]:
    radius = max(0.0, float(mu) * normal)
    lateral_cap = math.sqrt(max(0.0, radius * radius - fx * fx))
    force = float(np.clip(raw_fy, -lateral_cap, lateral_cap))
    saturated = abs(raw_fy) > lateral_cap + 1e-10
    utilization = math.hypot(fx, force) / max(radius, 1e-9)
    return force, saturated, utilization


def dynamic_bicycle_step(
    state: DynamicBicycleState,
    command: DynamicCommand,
    mu: float,
    cfg: DynamicBicycleConfig | None = None,
) -> DynamicStep:
    """Advance the planar dynamic bicycle with a stable hybrid integration.

    The linear-tire equations are stiff as longitudinal speed approaches zero.
    Below ``kinematic_blend_speed`` we therefore use their no-slip limit when
    the requested axle forces fit inside both friction circles.  Once the
    request reaches the grip boundary, or at normal driving speed, the same
    saturated dynamic bicycle is integrated in fixed substeps.  Consequently
    ``mu`` cannot affect a genuinely no-slip update.
    """
    cfg = cfg or DynamicBicycleConfig()
    dt = cfg.dt
    substeps = max(1, int(cfg.integration_substeps))
    h = dt / substeps
    speed_target = float(np.clip(command.speed, -cfg.v_reverse_max, cfg.v_max))
    steering_target = float(np.clip(command.steering, -cfg.max_steering_angle, cfg.max_steering_angle))
    # Preserve the legacy zero-order-hold actuator convention: rate-limit once
    # per public step, then integrate the resulting held steering angle.
    steering_applied = float(np.clip(
        steering_target,
        state.steering - cfg.max_steer_rate * dt,
        state.steering + cfg.max_steer_rate * dt,
    ))
    normal_front = cfg.mass * cfg.gravity * cfg.rear_length / cfg.wheelbase
    normal_rear = cfg.mass * cfg.gravity * cfg.front_length / cfg.wheelbase
    current = state.copy()
    supplied_work = 0.0
    dissipated = 0.0
    saturated = False
    max_utilization = 0.0
    longitudinal_force = front_force = rear_force = lateral_acceleration = 0.0
    drive_force = rolling = 0.0
    aero_body = np.zeros(3, dtype=np.float64)
    aero_point = np.zeros(3, dtype=np.float64)
    relative_air_body = np.zeros(3, dtype=np.float64)
    util_front = util_rear = 0.0

    for _ in range(substeps):
        steering = steering_applied
        requested_accel = float(np.clip(
            (speed_target - current.vx) / max(cfg.speed_time_constant, 1e-6),
            -cfg.max_brake, cfg.max_accel,
        ))
        requested_drive = cfg.mass * requested_accel
        rear_limit = max(0.0, float(mu) * normal_rear)
        drive_force = float(np.clip(requested_drive, -rear_limit, rear_limit))
        drive_saturated = abs(drive_force - requested_drive) > 1e-10
        rolling = cfg.rolling_resistance * math.tanh(current.vx / 0.05)
        c_heading, s_heading = math.cos(current.heading), math.sin(current.heading)
        planar_rotation = np.asarray([
            [c_heading, -s_heading, 0.0],
            [s_heading, c_heading, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        velocity_world = planar_rotation @ np.asarray([current.vx, current.vy, 0.0], dtype=np.float64)
        aero_body, aero_point, relative_air_body = aerodynamic_force_body(
            velocity_world, planar_rotation, cfg,
        )

        curvature = math.tan(steering) / cfg.wheelbase
        lateral_demand = cfg.mass * current.vx * current.vx * curvature
        desired_front = lateral_demand * cfg.rear_length / cfg.wheelbase
        desired_rear = lateral_demand * cfg.front_length / cfg.wheelbase
        no_slip_feasible = (
            abs(desired_front) <= float(mu) * normal_front + 1e-12
            and math.hypot(drive_force, desired_rear) <= float(mu) * normal_rear + 1e-12
            and abs(float(aero_body[1])) <= float(mu) * (normal_front + normal_rear) + 1e-12
        )
        use_kinematic = abs(current.vx) < cfg.kinematic_blend_speed and no_slip_feasible

        old_vx, old_vy, old_yaw = current.vx, current.vy, current.yaw_rate
        if use_kinematic:
            longitudinal_force = drive_force - rolling + float(aero_body[0])
            vx = old_vx + h * longitudinal_force / cfg.mass
            vy = 0.0
            yaw_rate = vx * curvature
            front_force, rear_force = desired_front, desired_rear
            lateral_acceleration = vx * vx * curvature + float(aero_body[1]) / cfg.mass
            util_front = abs(front_force) / max(float(mu) * normal_front, 1e-9)
            util_rear = math.hypot(drive_force, rear_force) / max(float(mu) * normal_rear, 1e-9)
            # The steering actuator establishes the no-slip yaw state.  Account
            # for that reversible rotational work at the external port.
            steering_work = 0.5 * cfg.yaw_inertia * (yaw_rate * yaw_rate - old_yaw * old_yaw)
        else:
            vx_ref = math.copysign(
                max(abs(old_vx), cfg.low_speed_regularizer),
                old_vx if old_vx != 0.0 else 1.0,
            )
            alpha_front = steering - math.atan2(old_vy + cfg.front_length * old_yaw, abs(vx_ref))
            alpha_rear = -math.atan2(old_vy - cfg.rear_length * old_yaw, abs(vx_ref))
            if vx_ref < 0.0:
                alpha_front, alpha_rear = -alpha_front, -alpha_rear
            raw_front = cfg.front_cornering_stiffness * alpha_front
            raw_rear = cfg.rear_cornering_stiffness * alpha_rear
            front_force, sat_front, util_front = _combined_grip_lateral(raw_front, normal_front, 0.0, mu)
            rear_force, sat_rear, util_rear = _combined_grip_lateral(raw_rear, normal_rear, drive_force, mu)
            body_fx = drive_force - front_force * math.sin(steering) - rolling + float(aero_body[0])
            body_fy = front_force * math.cos(steering) + rear_force + float(aero_body[1])
            longitudinal_force = body_fx
            dvx = body_fx / cfg.mass + old_yaw * old_vy
            dvy = body_fy / cfg.mass - old_yaw * old_vx
            dyaw = (
                cfg.front_length * front_force * math.cos(steering)
                - cfg.rear_length * rear_force
            ) / cfg.yaw_inertia
            vx = old_vx + h * dvx
            vy = old_vy + h * dvy
            yaw_rate = old_yaw + h * dyaw
            lateral_acceleration = body_fy / cfg.mass
            slip_front = (old_vy + cfg.front_length * old_yaw) * math.cos(steering) - old_vx * math.sin(steering)
            slip_rear = old_vy - cfg.rear_length * old_yaw
            dissipated += h * (
                max(0.0, -front_force * slip_front)
                + max(0.0, -rear_force * slip_rear)
            )
            saturated = saturated or sat_front or sat_rear
            steering_work = 0.0

        if abs(vx) < 0.015 and abs(speed_target) < 0.015:
            vx = vy = yaw_rate = 0.0
        avg_vx = 0.5 * (old_vx + vx)
        supplied_work += drive_force * avg_vx * h + steering_work
        dissipated += max(0.0, rolling * avg_vx - float(np.dot(aero_body, relative_air_body))) * h
        saturated = saturated or drive_saturated
        max_utilization = max(max_utilization, util_front, util_rear)

        avg_yaw = 0.5 * (old_yaw + yaw_rate)
        mid_heading = current.heading + 0.5 * h * avg_yaw
        c, s = math.cos(mid_heading), math.sin(mid_heading)
        current = DynamicBicycleState(
            x=current.x + h * (vx * c - vy * s),
            y=current.y + h * (vx * s + vy * c),
            heading=wrap_angle(current.heading + h * avg_yaw),
            vx=vx, vy=vy, yaw_rate=yaw_rate, steering=steering,
        )

    next_state = current
    kinetic = (
        0.5 * cfg.mass * (current.vx * current.vx + current.vy * current.vy)
        + 0.5 * cfg.yaw_inertia * current.yaw_rate * current.yaw_rate
    )
    # The planar plant does not integrate vertical motion, but exposing the
    # balanced gravity/normal forces makes the model useful as a force-aware
    # digital twin.  Positions use the same body convention as the simulator:
    # x forward, y left, z up, with the center of mass at the origin.
    front_normal = float(normal_front)
    rear_normal = float(normal_rear)
    front_body = np.asarray([-front_force * math.sin(steering_applied),
                             front_force * math.cos(steering_applied), 0.0], dtype=np.float64)
    rear_body = np.asarray([0.0, rear_force, 0.0], dtype=np.float64)
    rear_drive = np.asarray([drive_force, 0.0, 0.0], dtype=np.float64)
    rolling_body = np.asarray([-rolling, 0.0, 0.0], dtype=np.float64)
    drag_body = aero_body
    gravity_body = np.asarray([0.0, 0.0, -cfg.mass * cfg.gravity], dtype=np.float64)
    front_normal_body = np.asarray([0.0, 0.0, front_normal], dtype=np.float64)
    rear_normal_body = np.asarray([0.0, 0.0, rear_normal], dtype=np.float64)
    total_body = front_body + rear_body + rear_drive + rolling_body + drag_body
    total_body += gravity_body + front_normal_body + rear_normal_body
    total_moment = np.asarray([
        0.0,
        0.0,
        cfg.front_length * front_force * math.cos(steering_applied)
        - cfg.rear_length * rear_force,
    ], dtype=np.float64)
    total_moment += np.cross(aero_point, aero_body)

    def component(vector: np.ndarray, position: tuple[float, float, float], *, enabled: bool = True,
                  utilization: float | None = None) -> dict[str, Any]:
        return {
            "force_body": vector.tolist(),
            "application_point_body": list(position),
            "enabled": bool(enabled),
            **({"utilization": float(utilization)} if utilization is not None else {}),
        }

    breakdown = {
        "gravity": component(gravity_body, (0.0, 0.0, 0.0)),
        "normal_front": component(front_normal_body, (cfg.front_length, 0.0, 0.0)),
        "normal_rear": component(rear_normal_body, (-cfg.rear_length, 0.0, 0.0)),
        "tire_front_lateral": component(front_body, (cfg.front_length, 0.0, 0.0),
                                         utilization=util_front),
        "tire_rear_lateral": component(rear_body, (-cfg.rear_length, 0.0, 0.0),
                                        utilization=util_rear),
        "tire_rear_longitudinal": component(rear_drive, (-cfg.rear_length, 0.0, 0.0),
                                             utilization=util_rear),
        "rolling_resistance": component(rolling_body, (0.0, 0.0, 0.0),
                                         enabled=abs(rolling) > 1e-12),
        "aerodynamic_drag": component(drag_body, tuple(aero_point.tolist()),
                                       enabled=float(np.linalg.norm(drag_body)) > 1e-12),
        "total_external": {
            "force_body": total_body.tolist(),
            "moment_body": total_moment.tolist(),
            "enabled": True,
        },
    }
    return DynamicStep(
        state=next_state,
        lateral_acceleration=float(lateral_acceleration),
        longitudinal_force=float(longitudinal_force),
        front_lateral_force=float(front_force),
        rear_lateral_force=float(rear_force),
        supplied_work=float(supplied_work),
        dissipated_energy=float(dissipated),
        kinetic_energy=float(kinetic),
        saturated=bool(saturated),
        friction_utilization=float(max_utilization),
        force_breakdown=breakdown,
    )


@dataclass(frozen=True)
class FrictionMeasurement:
    pose: np.ndarray
    yaw_rate: float | None = None
    lateral_acceleration: float | None = None


@dataclass
class FrictionBelief:
    mean: float
    lower: float
    variance: float
    entropy: float
    innovation: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float64))
    covariance: np.ndarray = field(default_factory=lambda: np.zeros((0, 0), dtype=np.float64))
    nis: float = 0.0
    reset: bool = False


class BayesianFrictionEstimator:
    """Auditable discrete Bayes filter for a piecewise-constant surface ``mu``."""

    def __init__(
        self,
        cfg: DynamicBicycleConfig | None = None,
        grid: Iterable[float] | None = None,
        pose_std: tuple[float, float, float] = (0.025, 0.025, 0.015),
        yaw_rate_std: float = 0.04,
        lateral_accel_std: float = 0.12,
        lower_quantile: float = 0.10,
        reset_nis_probability: float = 0.999,
        reset_streak: int = 4,
    ):
        self.cfg = cfg or DynamicBicycleConfig()
        self.grid = np.asarray(list(grid) if grid is not None else np.linspace(0.15, 1.20, 71), dtype=np.float64)
        if self.grid.ndim != 1 or len(self.grid) < 2 or np.any(np.diff(self.grid) <= 0.0):
            raise ValueError("friction grid must be a strictly increasing vector")
        self.pose_std = np.asarray(pose_std, dtype=np.float64)
        self.yaw_rate_std = float(yaw_rate_std)
        self.lateral_accel_std = float(lateral_accel_std)
        self.lower_quantile = float(lower_quantile)
        self.reset_nis_probability = float(reset_nis_probability)
        self.reset_streak = int(reset_streak)
        self.weights = np.full(len(self.grid), 1.0 / len(self.grid), dtype=np.float64)
        self.inconsistent_streak = 0
        self.nis_history: list[float] = []

    def reset(self) -> None:
        self.weights.fill(1.0 / len(self.weights))
        self.inconsistent_streak = 0
        self.nis_history.clear()

    def _belief(self, innovation: np.ndarray | None = None, covariance: np.ndarray | None = None,
                nis: float = 0.0, reset: bool = False) -> FrictionBelief:
        mean = float(np.dot(self.weights, self.grid))
        variance = float(np.dot(self.weights, (self.grid - mean) ** 2))
        cumulative = np.cumsum(self.weights)
        lower = float(self.grid[min(int(np.searchsorted(cumulative, self.lower_quantile)), len(self.grid) - 1)])
        entropy = float(-np.sum(self.weights * np.log(self.weights + 1e-15)))
        return FrictionBelief(
            mean, lower, variance, entropy,
            np.zeros(0) if innovation is None else innovation,
            np.zeros((0, 0)) if covariance is None else covariance,
            float(nis), bool(reset),
        )

    @property
    def belief(self) -> FrictionBelief:
        return self._belief()

    @staticmethod
    def _nis_threshold(dof: int, probability: float) -> float:
        # Wilson-Hilferty approximation avoids adding scipy to the project.
        z = NormalDist().inv_cdf(float(probability))
        return float(dof * (1.0 - 2.0 / (9.0 * dof) + z * math.sqrt(2.0 / (9.0 * dof))) ** 3)

    def update(
        self,
        previous_state: DynamicBicycleState,
        command: DynamicCommand,
        measurement: FrictionMeasurement,
        sensing_mode: str,
    ) -> FrictionBelief:
        use_imu = sensing_mode in {"imu", "imu_probe", "oracle"}
        predictions: list[np.ndarray] = []
        for mu in self.grid:
            step = dynamic_bicycle_step(previous_state, command, float(mu), self.cfg)
            values = list(step.state.pose)
            if use_imu:
                values.extend([step.state.yaw_rate, step.lateral_acceleration])
            predictions.append(np.asarray(values, dtype=np.float64))
        pred = np.stack(predictions)
        measured = list(np.asarray(measurement.pose, dtype=np.float64))
        std = list(self.pose_std)
        if use_imu:
            if measurement.yaw_rate is None or measurement.lateral_acceleration is None:
                raise ValueError("IMU sensing mode requires yaw rate and lateral acceleration")
            measured.extend([measurement.yaw_rate, measurement.lateral_acceleration])
            std.extend([self.yaw_rate_std, self.lateral_accel_std])
        measured_arr = np.asarray(measured, dtype=np.float64)
        std_arr = np.asarray(std, dtype=np.float64)
        residuals = measured_arr[None, :] - pred
        residuals[:, 2] = np.arctan2(np.sin(residuals[:, 2]), np.cos(residuals[:, 2]))
        log_likelihood = -0.5 * np.sum((residuals / std_arr[None, :]) ** 2, axis=1)
        log_weights = np.log(self.weights + 1e-300) + log_likelihood
        log_weights -= np.max(log_weights)
        self.weights = np.exp(log_weights)
        self.weights /= np.sum(self.weights)

        predicted_mean = np.sum(self.weights[:, None] * pred, axis=0)
        innovation = measured_arr - predicted_mean
        innovation[2] = wrap_angle(innovation[2])
        centered = pred - predicted_mean[None, :]
        centered[:, 2] = np.arctan2(np.sin(centered[:, 2]), np.cos(centered[:, 2]))
        covariance = (centered.T * self.weights) @ centered + np.diag(std_arr * std_arr)
        nis = float(innovation @ np.linalg.solve(covariance, innovation))
        self.nis_history.append(nis)
        inconsistent = nis > self._nis_threshold(len(innovation), self.reset_nis_probability)
        self.inconsistent_streak = self.inconsistent_streak + 1 if inconsistent else 0
        did_reset = self.inconsistent_streak >= self.reset_streak
        if did_reset:
            self.reset()
        return self._belief(innovation, covariance, nis, did_reset)

    def whiteness_score(self, window: int = 20) -> float:
        values = np.asarray(self.nis_history[-int(window):], dtype=np.float64)
        if len(values) < 3 or float(values.std()) < 1e-12:
            return 0.0
        return float(np.corrcoef(values[:-1], values[1:])[0, 1])


@dataclass
class PassivityAudit:
    tolerance: float = 0.35
    relative_tolerance: float = 0.05
    release_steps: int = 5
    previous_energy: float | None = None
    fallback_active: bool = False
    good_streak: int = 0
    last_threshold: float = 0.0

    def reset(self, energy: float | None = None) -> None:
        self.previous_energy = energy
        self.fallback_active = False
        self.good_streak = 0
        self.last_threshold = 0.0

    def update(self, energy: float, supplied_work: float, dissipated_energy: float) -> tuple[float, bool]:
        old_energy = self.previous_energy
        if old_energy is None:
            residual = 0.0
        else:
            residual = float(energy - old_energy - supplied_work + dissipated_energy)
        scale = max(
            abs(float(energy)),
            0.0 if old_energy is None else abs(float(old_energy)),
            abs(float(supplied_work)),
            abs(float(dissipated_energy)),
        )
        threshold = float(self.tolerance + self.relative_tolerance * scale)
        self.last_threshold = threshold
        self.previous_energy = float(energy)
        # Passivity constrains unexplained energy creation.  A negative residual
        # is additional (unmodeled) dissipation and must not trip the latch.
        if not np.isfinite(residual) or residual > threshold:
            self.fallback_active = True
            self.good_streak = 0
        elif self.fallback_active:
            self.good_streak += 1
            if self.good_streak >= self.release_steps:
                self.fallback_active = False
                self.good_streak = 0
        return residual, self.fallback_active


def state_flow_error(state: DynamicBicycleState, command: DynamicCommand, estimated_mu: float,
                     true_mu: float, cfg: DynamicBicycleConfig) -> float:
    estimate = dynamic_bicycle_step(state, command, estimated_mu, cfg).state
    truth = dynamic_bicycle_step(state, command, true_mu, cfg).state
    e = np.asarray([estimate.vx - truth.vx, estimate.vy - truth.vy, estimate.yaw_rate - truth.yaw_rate])
    denom = np.linalg.norm([truth.vx, truth.vy, truth.yaw_rate])
    return float(np.linalg.norm(e) / max(float(denom), 1e-6))


def jsonable_belief(belief: FrictionBelief) -> dict[str, Any]:
    return {
        "mean": belief.mean,
        "lower": belief.lower,
        "variance": belief.variance,
        "entropy": belief.entropy,
        "innovation": belief.innovation.tolist(),
        "covariance": belief.covariance.tolist(),
        "nis": belief.nis,
        "reset": belief.reset,
    }
