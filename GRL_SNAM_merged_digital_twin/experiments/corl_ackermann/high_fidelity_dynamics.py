"""Four-wheel, suspension-aware six-degree-of-freedom vehicle dynamics.

This is the next-fidelity digital-twin plant.  It is deliberately compact and
auditable: a rigid chassis, four independently saturated tire contacts,
vertical suspension loads, wheel rotational inertia, and diagonal chassis
inertia.  The original dynamic bicycle remains available for the research
benchmarks; this model is selected explicitly by the interactive viewer.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable

import numpy as np

from experiments.corl_ackermann.environment_dynamics import (
    DynamicBicycleConfig,
    aerodynamic_force_body,
    wrap_angle,
)


WHEEL_NAMES = ("fl", "fr", "rl", "rr")


@dataclass
class SixDofVehicleState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.17
    roll: float = 0.0
    pitch: float = 0.0
    heading: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0
    steering: float = 0.0
    omega_fl: float = 0.0
    omega_fr: float = 0.0
    omega_rl: float = 0.0
    omega_rr: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


@dataclass
class SixDofStep:
    state: SixDofVehicleState
    lateral_acceleration: float
    longitudinal_force: float
    front_lateral_force: float
    rear_lateral_force: float
    supplied_work: float
    dissipated_energy: float
    kinetic_energy: float
    saturated: bool
    friction_utilization: float
    force_breakdown: dict[str, dict[str, Any]] = field(default_factory=dict)


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    return q / max(float(np.linalg.norm(q)), 1e-12)


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.asarray([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def _rotation_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _quat_normalize(q)
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _euler_from_rotation(rotation: np.ndarray) -> tuple[float, float, float]:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return roll, pitch, wrap_angle(yaw)


def _wheel_omegas(state: SixDofVehicleState) -> list[float]:
    return [state.omega_fl, state.omega_fr, state.omega_rl, state.omega_rr]


def _set_wheel_omegas(state: SixDofVehicleState, values: list[float]) -> None:
    state.omega_fl, state.omega_fr, state.omega_rl, state.omega_rr = values


def _component(
    force_body: np.ndarray,
    point_body: np.ndarray,
    rotation: np.ndarray,
    *,
    enabled: bool = True,
    utilization: float | None = None,
    force_limit: float | None = None,
    slip_angle: float | None = None,
    slip_ratio: float | None = None,
) -> dict[str, Any]:
    value = {
        "force_body": force_body.tolist(),
        "force_world": (rotation @ force_body).tolist(),
        "application_point_body": point_body.tolist(),
        "enabled": bool(enabled),
    }
    if utilization is not None:
        value["utilization"] = float(utilization)
    if force_limit is not None:
        value["force_limit"] = float(force_limit)
    if slip_angle is not None:
        value["slip_angle"] = float(slip_angle)
    if slip_ratio is not None:
        value["slip_ratio"] = float(slip_ratio)
    return value


def initial_six_dof_state(cfg: DynamicBicycleConfig | None = None) -> SixDofVehicleState:
    cfg = cfg or DynamicBicycleConfig()
    contact_margin = max(0.01, cfg.wheel_radius * 0.45)
    return SixDofVehicleState(z=cfg.com_height + cfg.wheel_radius + contact_margin)


def six_dof_step(
    state: SixDofVehicleState,
    target_speed: float,
    steering_command: float,
    mu: float,
    cfg: DynamicBicycleConfig | None = None,
    ground_height_fn: Callable[[float, float], float] | None = None,
) -> SixDofStep:
    """Advance a four-wheel rigid-body vehicle over optional terrain."""
    cfg = cfg or DynamicBicycleConfig()
    dt = float(cfg.dt)
    substeps = max(1, int(cfg.integration_substeps))
    h = dt / substeps
    mu = max(float(mu), 1e-4)
    target_speed = float(np.clip(target_speed, -cfg.v_reverse_max, cfg.v_max))
    steering_target = float(np.clip(steering_command, -cfg.max_steering_angle, cfg.max_steering_angle))
    steering = float(np.clip(
        steering_target,
        state.steering - cfg.max_steer_rate * dt,
        state.steering + cfg.max_steer_rate * dt,
    ))
    normal_static = [
        cfg.mass * cfg.gravity * cfg.rear_length / cfg.wheelbase / 2.0,
        cfg.mass * cfg.gravity * cfg.rear_length / cfg.wheelbase / 2.0,
        cfg.mass * cfg.gravity * cfg.front_length / cfg.wheelbase / 2.0,
        cfg.mass * cfg.gravity * cfg.front_length / cfg.wheelbase / 2.0,
    ]
    wheel_points = [
        np.asarray([cfg.front_length, cfg.track_width / 2.0, -cfg.com_height], dtype=np.float64),
        np.asarray([cfg.front_length, -cfg.track_width / 2.0, -cfg.com_height], dtype=np.float64),
        np.asarray([-cfg.rear_length, cfg.track_width / 2.0, -cfg.com_height], dtype=np.float64),
        np.asarray([-cfg.rear_length, -cfg.track_width / 2.0, -cfg.com_height], dtype=np.float64),
    ]
    front = (True, True, False, False)
    drive_torques = [0.0, 0.0, 0.0, 0.0]
    latest: dict[str, Any] = {}
    total_force_body = np.zeros(3, dtype=np.float64)
    total_moment_body = np.zeros(3, dtype=np.float64)
    total_front_lateral = 0.0
    total_rear_lateral = 0.0
    total_longitudinal = 0.0
    max_utilization = 0.0
    saturated = False
    supplied_work = 0.0
    dissipated = 0.0
    wheel_omegas = _wheel_omegas(state)

    for _ in range(substeps):
        # Recompute the external wrench from the current substep state.  The
        # wheel angular velocities are carried across substeps, but forces and
        # moments are instantaneous quantities and must not accumulate.
        total_front_lateral = 0.0
        total_rear_lateral = 0.0
        total_longitudinal = 0.0
        q = _quat_normalize(np.asarray([state.qw, state.qx, state.qy, state.qz], dtype=np.float64))
        rotation = _rotation_matrix(q)
        velocity_body = np.asarray([state.vx, state.vy, state.vz], dtype=np.float64)
        angular_body = np.asarray([state.roll_rate, state.pitch_rate, state.yaw_rate], dtype=np.float64)
        wheel_forces: list[np.ndarray] = []
        wheel_normals: list[np.ndarray] = []
        wheel_utilizations: list[float] = []
        wheel_components: dict[str, dict[str, Any]] = {}
        forces_body = np.zeros(3, dtype=np.float64)
        moments_body = np.zeros(3, dtype=np.float64)
        requested_accel = float(np.clip(
            (target_speed - state.vx) / max(cfg.speed_time_constant, 1e-6),
            -cfg.max_brake, cfg.max_accel,
        ))
        requested_force = cfg.mass * requested_accel
        requested_rear_force = requested_force / 2.0
        for index, (name, point, is_front) in enumerate(zip(WHEEL_NAMES, wheel_points, front)):
            point_world = rotation @ point
            center_world_z = state.z + point_world[2]
            wheel_world_x = state.x + point_world[0]
            wheel_world_y = state.y + point_world[1]
            ground_height = (
                float(ground_height_fn(wheel_world_x, wheel_world_y))
                if ground_height_fn is not None else 0.0
            )
            contact_margin = max(0.01, cfg.wheel_radius * 0.45)
            contact_height = ground_height + cfg.wheel_radius + contact_margin
            contact = center_world_z <= contact_height + 1e-6
            compression = max(0.0, contact_height - center_world_z)
            contact_velocity_body = velocity_body + np.cross(angular_body, point)
            contact_velocity_world = rotation @ contact_velocity_body
            normal = max(
                0.0,
                normal_static[index] + cfg.suspension_stiffness * compression
                - cfg.suspension_damping * contact_velocity_world[2],
            ) if contact else 0.0
            if ground_height_fn is not None and normal > 0.0:
                sample_delta = 0.08
                slope_x = (
                    float(ground_height_fn(wheel_world_x + sample_delta, wheel_world_y))
                    - float(ground_height_fn(wheel_world_x - sample_delta, wheel_world_y))
                ) / (2.0 * sample_delta)
                slope_y = (
                    float(ground_height_fn(wheel_world_x, wheel_world_y + sample_delta))
                    - float(ground_height_fn(wheel_world_x, wheel_world_y - sample_delta))
                ) / (2.0 * sample_delta)
                normal_world = np.asarray([-slope_x * normal, -slope_y * normal, normal], dtype=np.float64)
            else:
                normal_world = np.asarray([0.0, 0.0, normal], dtype=np.float64)
            normal_body = rotation.T @ normal_world
            delta = steering if is_front else 0.0
            c_delta, s_delta = math.cos(delta), math.sin(delta)
            wheel_velocity_x = c_delta * contact_velocity_body[0] + s_delta * contact_velocity_body[1]
            wheel_velocity_y = -s_delta * contact_velocity_body[0] + c_delta * contact_velocity_body[1]
            effective_speed = max(abs(wheel_velocity_x), 0.20)
            slip_ratio = (cfg.wheel_radius * wheel_omegas[index] - wheel_velocity_x) / effective_speed
            slip_angle = math.atan2(wheel_velocity_y, effective_speed)
            raw_fx = cfg.tire_longitudinal_stiffness * slip_ratio
            cornering = cfg.front_cornering_stiffness if is_front else cfg.rear_cornering_stiffness
            raw_fy = -cornering * slip_angle
            if is_front:
                drive_torques[index] = 0.0
            else:
                # The speed command is the motor/brake request in this
                # interactive plant.  Track that requested axle force while
                # still charging the wheel inertia toward the rolling speed;
                # the friction circle below remains the hard physical limit.
                raw_fx = requested_rear_force
                desired_omega = wheel_velocity_x / max(cfg.wheel_radius, 1e-6)
                drive_torques[index] = float(np.clip(
                    raw_fx * cfg.wheel_radius
                    + cfg.wheel_inertia * (desired_omega - wheel_omegas[index]) / max(h, 1e-6),
                    -cfg.max_drive_torque,
                    cfg.max_drive_torque,
                ))
            grip_radius = mu * normal
            raw_norm = math.hypot(raw_fx, raw_fy)
            scale = min(1.0, grip_radius / max(raw_norm, 1e-12)) if grip_radius > 0 else 0.0
            fx = raw_fx * scale
            fy = raw_fy * scale
            utilization = math.hypot(fx, fy) / max(grip_radius, 1e-9)
            wheel_utilizations.append(utilization)
            max_utilization = max(max_utilization, utilization)
            saturated = saturated or scale < 1.0 - 1e-10
            force_wheel = np.asarray([c_delta * fx - s_delta * fy, s_delta * fx + c_delta * fy, 0.0])
            tire_body = force_wheel
            # The legacy bicycle parameter is a vehicle-level force scale in
            # newtons.  Distribute that force by normal load here so the same
            # preset does not accidentally become a 10% tire-load coefficient.
            rolling = (
                cfg.rolling_resistance * normal / max(cfg.mass * cfg.gravity, 1e-9)
                * math.tanh(wheel_velocity_x / 0.05)
            )
            rolling_body = np.asarray([-rolling * c_delta, -rolling * s_delta, 0.0])
            total_wheel_body = tire_body + rolling_body + normal_body
            force_wheel_point = point
            forces_body += total_wheel_body
            moments_body += np.cross(force_wheel_point, total_wheel_body)
            wheel_forces.append(tire_body)
            wheel_normals.append(normal_body)
            wheel_name = name
            wheel_components[f"tire_{wheel_name}"] = _component(
                tire_body,
                point,
                rotation,
                enabled=normal > 0.0,
                utilization=utilization,
                force_limit=grip_radius,
                slip_angle=slip_angle,
                slip_ratio=slip_ratio,
            )
            wheel_components[f"suspension_{wheel_name}"] = _component(
                normal_body, point, rotation, enabled=normal > 0.0,
            )
            wheel_components[f"rolling_{wheel_name}"] = _component(
                rolling_body, point, rotation, enabled=abs(rolling) > 1e-12,
            )
            wheel_omegas[index] += h * (
                drive_torques[index] - fx * cfg.wheel_radius
                - rolling * cfg.wheel_radius * math.copysign(1.0, wheel_omegas[index] or 1.0)
            ) / max(cfg.wheel_inertia, 1e-6)
            supplied_work += drive_torques[index] * wheel_omegas[index] * h
            dissipated += max(0.0, abs(fx * (cfg.wheel_radius * wheel_omegas[index] - wheel_velocity_x))) * h
            total_front_lateral += fy if is_front else 0.0
            total_rear_lateral += fy if not is_front else 0.0
            total_longitudinal += fx + rolling

        gravity_world = np.asarray([0.0, 0.0, -cfg.mass * cfg.gravity], dtype=np.float64)
        gravity_body = rotation.T @ gravity_world
        aero_body, aero_point, _relative_air_body = aerodynamic_force_body(
            rotation @ velocity_body,
            rotation,
            cfg,
        )
        forces_body += gravity_body + aero_body
        moments_body += np.cross(aero_point, aero_body)
        wheel_components["gravity"] = _component(gravity_body, np.zeros(3), rotation)
        wheel_components["aerodynamic_drag"] = _component(
            aero_body, aero_point, rotation, enabled=float(np.linalg.norm(aero_body)) > 1e-12,
        )
        inertia = np.asarray([cfg.roll_inertia, cfg.pitch_inertia, cfg.yaw_inertia], dtype=np.float64)
        coriolis_linear = np.cross(angular_body, velocity_body)
        acceleration_body = forces_body / max(cfg.mass, 1e-9) - coriolis_linear
        angular_acceleration = (
            moments_body - np.cross(angular_body, inertia * angular_body)
        ) / np.maximum(inertia, 1e-9)
        velocity_body += h * acceleration_body
        angular_body += h * angular_acceleration
        position_world = np.asarray([state.x, state.y, state.z], dtype=np.float64)
        position_world += h * (rotation @ velocity_body)
        q += h * 0.5 * _quat_multiply(q, np.asarray([0.0, *angular_body]))
        q = _quat_normalize(q)
        rotation_next = _rotation_matrix(q)
        roll, pitch, yaw = _euler_from_rotation(rotation_next)
        state.x, state.y, state.z = position_world.tolist()
        state.vx, state.vy, state.vz = velocity_body.tolist()
        state.roll_rate, state.pitch_rate, state.yaw_rate = angular_body.tolist()
        state.roll, state.pitch, state.heading = roll, pitch, yaw
        state.qw, state.qx, state.qy, state.qz = q.tolist()
        state.steering = steering

        # Prevent numerical drift from putting the chassis below the flat road.
        terrain_height = (
            float(ground_height_fn(state.x, state.y))
            if ground_height_fn is not None else 0.0
        )
        minimum_z = terrain_height + cfg.com_height + cfg.wheel_radius * 0.55
        if state.z < minimum_z:
            state.z = minimum_z
            state.vz = max(0.0, state.vz)
        latest = wheel_components

    total_force_body = forces_body.copy()
    total_moment_body = moments_body.copy()
    latest["total_external"] = {
        "force_body": total_force_body.tolist(),
        "force_world": (_rotation_matrix(np.asarray([state.qw, state.qx, state.qy, state.qz])) @ total_force_body).tolist(),
        "moment_body": total_moment_body.tolist(),
        "enabled": True,
    }
    kinetic = (
        0.5 * cfg.mass * (state.vx ** 2 + state.vy ** 2 + state.vz ** 2)
        + 0.5 * cfg.roll_inertia * state.roll_rate ** 2
        + 0.5 * cfg.pitch_inertia * state.pitch_rate ** 2
        + 0.5 * cfg.yaw_inertia * state.yaw_rate ** 2
        + 0.5 * cfg.wheel_inertia * sum(value * value for value in wheel_omegas)
    )
    _set_wheel_omegas(state, wheel_omegas)
    return SixDofStep(
        state=state,
        lateral_acceleration=float(total_force_body[1] / max(cfg.mass, 1e-9)),
        longitudinal_force=float(total_force_body[0]),
        front_lateral_force=float(total_front_lateral),
        rear_lateral_force=float(total_rear_lateral),
        supplied_work=float(supplied_work),
        dissipated_energy=float(dissipated),
        kinetic_energy=float(kinetic),
        saturated=bool(saturated),
        friction_utilization=float(max_utilization),
        force_breakdown=latest,
    )
