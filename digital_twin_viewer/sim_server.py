#!/usr/bin/env python3
"""Small dependency-free HTTP/WebSocket server for the digital-twin viewer.

The browser remains a renderer and control surface.  This process owns the
vehicle state and calls the same dynamic-bicycle plant used by the Ackermann
experiments.  It intentionally implements the small WebSocket subset needed
by this local tool so the viewer does not require an additional Python
dependency.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import csv
from dataclasses import asdict, fields, replace
import hashlib
import json
import math
import mimetypes
from pathlib import Path
import struct
import sys
from typing import Any
from urllib.parse import unquote, urlsplit


VIEWER_ROOT = Path(__file__).resolve().parent
REPO_ROOT = VIEWER_ROOT.parent / "GRL_SNAM_merged_digital_twin"
F1TENTH_MAP_ROOT = VIEWER_ROOT.parent / "f1tenth_benchmarks" / "maps"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.corl_ackermann.environment_dynamics import (  # noqa: E402
    DynamicBicycleConfig,
    DynamicBicycleState,
    DynamicCommand,
    dynamic_bicycle_step,
)
from experiments.corl_ackermann.high_fidelity_dynamics import (  # noqa: E402
    SixDofVehicleState,
    initial_six_dof_state,
    six_dof_step,
)


WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
EDITABLE_FIELDS = {
    field.name for field in fields(DynamicBicycleConfig)
    if field.name not in {"dt", "integration_substeps", "kinematic_blend_speed"}
}
PARAMETER_LIMITS = {
    "mass": (0.05, 500.0),
    "yaw_inertia": (1e-4, 500.0),
    "wheelbase": (0.05, 10.0),
    "front_length": (0.01, 10.0),
    "rear_length": (0.01, 10.0),
    "front_cornering_stiffness": (0.0, 10000.0),
    "rear_cornering_stiffness": (0.0, 10000.0),
    "gravity": (0.0, 30.0),
    "max_accel": (0.0, 100.0),
    "max_brake": (0.0, 100.0),
    "max_steer_rate": (0.0, 100.0),
    "max_steering_angle": (0.0, 1.5),
    "v_max": (0.0, 100.0),
    "v_reverse_max": (0.0, 100.0),
    "speed_time_constant": (0.01, 20.0),
    "aero_drag": (0.0, 3.0),
    "aero_side_drag": (0.0, 4.0),
    "air_density": (0.0, 3.0),
    "wind_speed": (0.0, 40.0),
    "wind_direction_deg": (-360.0, 360.0),
    "rolling_resistance": (0.0, 1000.0),
    "track_width": (0.03, 10.0),
    "body_length": (0.05, 20.0),
    "body_width": (0.03, 10.0),
    "body_height": (0.01, 10.0),
    "wheel_radius": (0.005, 5.0),
    "wheel_inertia": (1e-5, 100.0),
    "tire_longitudinal_stiffness": (0.0, 10000.0),
    "suspension_stiffness": (0.0, 100000.0),
    "suspension_damping": (0.0, 10000.0),
    "com_height": (0.01, 10.0),
    "roll_inertia": (1e-4, 500.0),
    "pitch_inertia": (1e-4, 500.0),
    "max_drive_torque": (0.0, 10000.0),
}
F1TENTH_TRACKS = {
    "aut": "Austria",
    "esp": "Barcelona",
    "gbr": "Great Britain",
    "mco": "Monaco",
}
TERRAIN_PRESETS = {
    "asphalt": {
        "label": "Dry asphalt",
        "friction_scale": 1.00,
        "rolling_scale": 1.00,
        "height_amplitude": 0.000,
    },
    "wet-asphalt": {
        "label": "Wet asphalt",
        "friction_scale": 0.72,
        "rolling_scale": 1.25,
        "height_amplitude": 0.004,
    },
    "gravel": {
        "label": "Gravel",
        "friction_scale": 0.58,
        "rolling_scale": 2.80,
        "height_amplitude": 0.025,
    },
    "grass": {
        "label": "Grass",
        "friction_scale": 0.45,
        "rolling_scale": 3.50,
        "height_amplitude": 0.040,
    },
    "mud": {
        "label": "Mud",
        "friction_scale": 0.32,
        "rolling_scale": 5.50,
        "height_amplitude": 0.060,
    },
    "ice": {
        "label": "Ice",
        "friction_scale": 0.14,
        "rolling_scale": 0.80,
        "height_amplitude": 0.003,
    },
}
_F1TENTH_PATH_CACHE: dict[str, Any] | None = None


def jsonable(value: Any) -> Any:
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {str(key): jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(item) for item in value]
    return value


def clamp_parameter(name: str, value: Any) -> Any:
    if name == "integration_substeps":
        return max(1, min(32, int(value)))
    numeric = float(value)
    lower, upper = PARAMETER_LIMITS[name]
    return max(lower, min(upper, numeric))


def config_with_values(base: DynamicBicycleConfig, values: dict[str, Any]) -> DynamicBicycleConfig:
    """Apply a model profile without mutating the active simulation config."""
    updates: dict[str, Any] = {}
    for name, value in values.items():
        if name in EDITABLE_FIELDS and name in PARAMETER_LIMITS:
            updates[name] = clamp_parameter(name, value)
    if "wheelbase" in updates and "front_length" not in updates and "rear_length" not in updates:
        updates["front_length"] = updates["wheelbase"] * 0.5
        updates["rear_length"] = updates["wheelbase"] * 0.5
    elif "front_length" in updates and "rear_length" in updates:
        updates["wheelbase"] = updates["front_length"] + updates["rear_length"]
    elif "front_length" in updates:
        updates["wheelbase"] = updates["front_length"] + base.rear_length
    elif "rear_length" in updates:
        updates["wheelbase"] = base.front_length + updates["rear_length"]
    return replace(base, **updates) if updates else base


def f1tenth_path_catalog() -> dict[str, Any]:
    """Expose the repository's metric F1TENTH centerlines to the browser."""
    global _F1TENTH_PATH_CACHE
    if _F1TENTH_PATH_CACHE is not None:
        return _F1TENTH_PATH_CACHE

    tracks: dict[str, Any] = {}
    for track_id, display_name in F1TENTH_TRACKS.items():
        source = F1TENTH_MAP_ROOT / f"{track_id}_centerline.csv"
        rows: list[list[float]] = []
        try:
            with source.open(newline="", encoding="utf-8") as handle:
                for row in csv.reader(handle):
                    if len(row) >= 4:
                        rows.append([float(value) for value in row[:4]])
        except (OSError, ValueError):
            continue
        if len(rows) < 3:
            continue

        points: list[dict[str, float]] = []
        total_length = 0.0
        lengths: list[float] = []
        for index, row in enumerate(rows):
            previous = rows[(index - 1) % len(rows)]
            following = rows[(index + 1) % len(rows)]
            dx = following[0] - previous[0]
            dy = following[1] - previous[1]
            heading = math.atan2(dy, dx)
            before = rows[index - 1]
            segment = math.hypot(row[0] - before[0], row[1] - before[1])
            lengths.append(segment)
            points.append({
                "x": row[0],
                "y": row[1],
                "width_left": row[2],
                "width_right": row[3],
                "heading": heading,
                "s": 0.0,
            })
        for index, segment in enumerate(lengths):
            points[index]["s"] = total_length
            total_length += segment
        for index, point in enumerate(points):
            previous = points[(index - 1) % len(points)]
            following = points[(index + 1) % len(points)]
            ds = max(lengths[index] + lengths[(index + 1) % len(lengths)], 1e-6)
            heading_delta = math.atan2(
                math.sin(following["heading"] - previous["heading"]),
                math.cos(following["heading"] - previous["heading"]),
            )
            point["curvature"] = heading_delta / ds

        # The benchmark racelines use a curvature-constrained velocity
        # profile instead of a constant throttle.  The repository checkout
        # contains the centerlines but not the generated raceline CSVs, so
        # reproduce the important part here: lateral grip first, followed by
        # forward/backward longitudinal acceleration passes.  The frontend
        # scales this reference profile by the selected vehicle's v_max and
        # the live friction coefficient.
        benchmark_v_max = 8.0
        benchmark_lateral_acc = 8.5
        benchmark_longitudinal_acc = 8.5
        benchmark_mu = 0.70
        lateral_acc_limit = min(benchmark_lateral_acc, benchmark_mu * 9.81)
        speed_profile = [
            min(
                benchmark_v_max,
                math.sqrt(lateral_acc_limit / max(abs(point["curvature"]), 1e-3)),
            )
            for point in points
        ]
        for _ in range(3):
            for index in range(len(points)):
                next_index = (index + 1) % len(points)
                segment = max(lengths[next_index], 1e-4)
                reachable = math.sqrt(
                    max(0.0, speed_profile[index] ** 2 + 2.0 * benchmark_longitudinal_acc * segment)
                )
                speed_profile[next_index] = min(speed_profile[next_index], reachable)
            for index in range(len(points) - 1, -1, -1):
                next_index = (index + 1) % len(points)
                segment = max(lengths[next_index], 1e-4)
                reachable = math.sqrt(
                    max(0.0, speed_profile[next_index] ** 2 + 2.0 * benchmark_longitudinal_acc * segment)
                )
                speed_profile[index] = min(speed_profile[index], reachable)
        for point, speed in zip(points, speed_profile):
            point["benchmark_speed"] = speed
        tracks[track_id] = {
            "id": track_id,
            "name": display_name,
            "closed": True,
            "length": total_length,
            "point_spacing": total_length / len(points),
            "vehicle_reference": {"length": 0.58, "width": 0.31, "wheelbase": 0.33},
            "speed_profile": {
                "max_speed": benchmark_v_max,
                "lateral_acceleration": benchmark_lateral_acc,
                "longitudinal_acceleration": benchmark_longitudinal_acc,
                "mu": benchmark_mu,
            },
            "points": points,
        }
    _F1TENTH_PATH_CACHE = {"type": "f1tenth_paths", "tracks": tracks}
    return _F1TENTH_PATH_CACHE


class SimulationSession:
    def __init__(self) -> None:
        # A 30 Hz viewer step is responsive while preserving the original
        # experiment defaults everywhere else.
        self.cfg = replace(DynamicBicycleConfig(), dt=1.0 / 30.0, integration_substeps=4)
        self.physics_model = "six_dof"
        self.vehicle_model = "race"
        self.state: Any = initial_six_dof_state(self.cfg)
        self.command = DynamicCommand(0.0, 0.0)
        self.mu = 0.70
        self.terrain_id = "asphalt"
        self.playing = True
        self.pose_ready = False
        self.sim_time = 0.0
        self.last_step: Any = None

    def reset(self) -> None:
        if self.physics_model == "six_dof":
            self.state = initial_six_dof_state(self.cfg)
        else:
            self.state = DynamicBicycleState()
        # Do not carry a cornering command through a reset. The browser sends
        # the track-start pose immediately afterward; until that pose arrives,
        # the controller must not steer from the temporary origin state.
        self.command = DynamicCommand(0.0, 0.0)
        self.pose_ready = False
        self.sim_time = 0.0
        self.last_step = None

    def update_parameters(self, values: dict[str, Any]) -> None:
        self.cfg = config_with_values(self.cfg, values)

    def terrain_preset(self) -> dict[str, Any]:
        return TERRAIN_PRESETS.get(self.terrain_id, TERRAIN_PRESETS["asphalt"])

    def terrain_preset_for(self, terrain_id: str | None) -> dict[str, Any]:
        return TERRAIN_PRESETS.get(str(terrain_id), TERRAIN_PRESETS["asphalt"])

    def effective_mu(self) -> float:
        return self.effective_mu_for(self.terrain_id)

    def effective_mu_for(self, terrain_id: str | None) -> float:
        preset = self.terrain_preset_for(terrain_id)
        return max(0.01, min(2.0, self.mu * float(preset["friction_scale"])))

    def terrain_config(self, cfg: DynamicBicycleConfig | None = None) -> DynamicBicycleConfig:
        return self.terrain_config_for(self.terrain_id, cfg)

    def terrain_config_for(
        self,
        terrain_id: str | None,
        cfg: DynamicBicycleConfig | None = None,
    ) -> DynamicBicycleConfig:
        base = cfg or self.cfg
        preset = self.terrain_preset_for(terrain_id)
        return config_with_values(base, {
            "rolling_resistance": base.rolling_resistance * float(preset["rolling_scale"]),
        })

    def terrain_height(self, x: float, y: float) -> float:
        return self.terrain_height_for(x, y, self.terrain_id)

    def terrain_height_for(self, x: float, y: float, terrain_id: str | None) -> float:
        amplitude = float(self.terrain_preset_for(terrain_id)["height_amplitude"])
        if amplitude <= 0.0:
            return 0.0
        # Deterministic multi-scale bumps give rough surfaces visible in the
        # six-DOF suspension response without requiring an external heightmap.
        return amplitude * (
            0.52 * math.sin(0.23 * x + 0.17 * y)
            + 0.30 * math.sin(0.61 * x - 0.37 * y + 1.4)
            + 0.18 * math.sin(1.65 * x + 1.21 * y)
        )

    def advance(self) -> None:
        if not self.playing:
            return
        if self.physics_model == "six_dof":
            self.last_step = six_dof_step(
                self.state,
                self.command.speed,
                self.command.steering,
                self.effective_mu(),
                self.terrain_config(),
                self.terrain_height,
            )
        else:
            self.last_step = dynamic_bicycle_step(
                self.state, self.command, self.effective_mu(), self.terrain_config(),
            )
        self.state = self.last_step.state
        self.sim_time += self.cfg.dt

    def snapshot(self) -> dict[str, Any]:
        state = asdict(self.state)
        breakdown = {} if self.last_step is None else self.last_step.force_breakdown
        return {
            "type": "snapshot",
            "time": self.sim_time,
            "playing": self.playing,
            "pose_ready": self.pose_ready,
            "physics_model": self.physics_model,
            "vehicle_model": self.vehicle_model,
            "vehicle": asdict(self.cfg),
            "mu": self.effective_mu(),
            "surface_mu": self.mu,
            "terrain": {
                "id": self.terrain_id,
                **self.terrain_preset(),
            },
            "wind": {
                "speed": float(self.cfg.wind_speed),
                "direction_deg": float(self.cfg.wind_direction_deg),
            },
            "command": {"target_speed": self.command.speed, "steering": self.command.steering},
            "state": state,
            "diagnostics": {
                "lateral_acceleration": 0.0 if self.last_step is None else self.last_step.lateral_acceleration,
                "longitudinal_force": 0.0 if self.last_step is None else self.last_step.longitudinal_force,
                "front_lateral_force": 0.0 if self.last_step is None else self.last_step.front_lateral_force,
                "rear_lateral_force": 0.0 if self.last_step is None else self.last_step.rear_lateral_force,
                "friction_utilization": 0.0 if self.last_step is None else self.last_step.friction_utilization,
                "saturated": False if self.last_step is None else self.last_step.saturated,
                "kinetic_energy": 0.0 if self.last_step is None else self.last_step.kinetic_energy,
                "supplied_work": 0.0 if self.last_step is None else self.last_step.supplied_work,
                "dissipated_energy": 0.0 if self.last_step is None else self.last_step.dissipated_energy,
                "force_breakdown": breakdown,
            },
        }

    def comparison(self, message: dict[str, Any]) -> dict[str, Any]:
        profiles = message.get("vehicle_profiles", {})
        variants: list[dict[str, str]] = []
        raw_variants = message.get("variants", [])
        if isinstance(raw_variants, list) and raw_variants:
            for value in raw_variants[:8]:
                if not isinstance(value, dict):
                    continue
                model_id = str(value.get("model_id", self.vehicle_model))
                terrain_id = str(value.get("terrain_id", self.terrain_id))
                variants.append({
                    "model_id": model_id,
                    "terrain_id": terrain_id if terrain_id in TERRAIN_PRESETS else self.terrain_id,
                    "label": str(value.get("label", "")),
                })
        else:
            model_ids = []
            for value in message.get("models", []):
                model_id = str(value)
                if model_id not in model_ids:
                    model_ids.append(model_id)
            terrain_ids = message.get("terrain_ids", [])
            for index, model_id in enumerate(model_ids[:8]):
                terrain_id = (
                    str(terrain_ids[index])
                    if isinstance(terrain_ids, list) and index < len(terrain_ids)
                    else self.terrain_id
                )
                variants.append({
                    "model_id": model_id,
                    "terrain_id": terrain_id if terrain_id in TERRAIN_PRESETS else self.terrain_id,
                    "label": "",
                })
        requested_steps = max(1, min(1800, int(message.get("steps", 900))))
        target_speed = float(message.get("target_speed", self.command.speed))
        steering = float(message.get("steering", self.command.steering))

        # Compare the selected vehicles on the same metric F1TENTH track.
        # This keeps the comparison meaningful: each vehicle sees identical
        # geometry and friction, but its own mass, dimensions, limits, and
        # tire parameters determine the resulting trajectory.
        track_id = str(message.get("track_id", ""))
        track = f1tenth_path_catalog().get("tracks", {}).get(track_id)
        if track and bool(message.get("follow_path", False)):
            path_points = track["points"]
            spacing = max(float(track.get("point_spacing", 0.2)), 0.05)
            benchmark_mu = float(track.get("speed_profile", {}).get("mu", 0.7))
            speed_ceiling = max(0.1, target_speed if target_speed > 0 else 8.0)
            # A comparison is an overlay of complete laps.  Give slower
            # presets enough simulation time to reach the finish, while
            # keeping a hard ceiling so a malformed controller cannot hang
            # the WebSocket request.
            max_steps = max(
                requested_steps,
                min(24000, math.ceil(float(track["length"]) / 0.35 / self.cfg.dt)),
            )
            start = path_points[0]
            path_trajectories = []
            for variant in variants:
                model_id = variant["model_id"]
                terrain_id = variant["terrain_id"]
                profile = profiles.get(model_id, {}) if isinstance(profiles, dict) else {}
                cfg = self.terrain_config_for(
                    terrain_id,
                    config_with_values(self.cfg, profile if isinstance(profile, dict) else {})
                )
                # Use the planar plant for the overlay.  It still consumes
                # every selected model's mass, geometry, tire, drag, and
                # speed parameters, but cannot roll over and terminate a
                # comparison before the two colored paths are comparable.
                state: Any = DynamicBicycleState()
                state.x = float(start["x"])
                state.y = float(start["y"])
                state.heading = float(start["heading"])
                state.vx = 0.0
                state.vy = 0.0
                state.yaw_rate = 0.0
                state.steering = 0.0
                points = []
                progress_index = 0
                lap_distance = 0.0
                distance_travelled = 0.0
                max_speed = 0.0
                min_track_margin = float("inf")
                stable = True
                for step_index in range(max_steps):
                    previous_progress_index = progress_index
                    candidates = [
                        (progress_index + offset) % len(path_points)
                        for offset in range(-30, 181)
                    ]
                    nearest_index = min(
                        candidates,
                        key=lambda index: math.hypot(
                            path_points[index]["x"] - state.x,
                            path_points[index]["y"] - state.y,
                        ),
                    )
                    forward_delta = (nearest_index - previous_progress_index) % len(path_points)
                    if 0 < forward_delta <= len(path_points) // 2:
                        lap_distance += forward_delta * spacing
                    progress_index = nearest_index
                    nearest = path_points[nearest_index]
                    speed = max(0.0, abs(float(getattr(state, "vx", 0.0))))
                    lookahead = max(0.65, min(2.8, 0.65 + speed * 0.22))
                    lookahead_index = (
                        nearest_index + max(2, round(lookahead / spacing))
                    ) % len(path_points)
                    target_point = path_points[lookahead_index]
                    target_angle = math.atan2(
                        target_point["y"] - state.y,
                        target_point["x"] - state.x,
                    )
                    alpha = math.atan2(
                        math.sin(target_angle - state.heading),
                        math.cos(target_angle - state.heading),
                    )
                    wheelbase = max(float(cfg.wheelbase), 0.05)
                    pure_pursuit = math.atan2(
                        2.0 * wheelbase * math.sin(alpha), lookahead,
                    )
                    heading_error = math.atan2(
                        math.sin(float(nearest["heading"]) - state.heading),
                        math.cos(float(nearest["heading"]) - state.heading),
                    )
                    lateral_error = (
                        -math.sin(float(nearest["heading"])) * (state.x - nearest["x"])
                        + math.cos(float(nearest["heading"])) * (state.y - nearest["y"])
                    )
                    steering_command = pure_pursuit + 0.25 * heading_error - math.atan2(
                        2.0 * lateral_error,
                        speed + 0.5,
                    )
                    steering_command = max(
                        -cfg.max_steering_angle,
                        min(cfg.max_steering_angle, steering_command),
                    )
                    preview_points = max(2, math.ceil(1.5 / spacing))
                    preview_speed = min(
                        float(path_points[(nearest_index + offset) % len(path_points)].get("benchmark_speed", 8.0))
                        for offset in range(preview_points + 1)
                    )
                    steering_speed_limit = (
                        8.0
                        if abs(steering_command) < 0.03
                        else math.sqrt(
                            1.5 * 9.81 * wheelbase
                            / max(math.tan(abs(steering_command)), 1e-3)
                        )
                    )
                    comparison_mu = self.effective_mu_for(terrain_id)
                    grip_scale = math.sqrt(max(0.1, comparison_mu) / benchmark_mu)
                    path_speed = min(preview_speed, steering_speed_limit, 8.0)
                    # Preserve a visible handling difference for tall/heavy
                    # presets without making the planar comparison stall.
                    stability_scale = min(
                        1.0,
                        max(0.45, (0.074 / max(float(cfg.com_height), 0.074)) ** 0.75),
                    )
                    path_speed = min(
                        speed_ceiling,
                        path_speed * grip_scale * 0.70 * stability_scale,
                        max(float(cfg.v_max), 0.0),
                    )
                    before_x, before_y = state.x, state.y
                    try:
                        result = dynamic_bicycle_step(
                            state,
                            DynamicCommand(path_speed, steering_command),
                            comparison_mu,
                            cfg,
                        )
                    except (OverflowError, FloatingPointError, ValueError):
                        stable = False
                        break
                    state = result.state
                    state_values = (
                        state.x,
                        state.y,
                        state.heading,
                        getattr(state, "z", 0.0),
                        getattr(state, "vx", 0.0),
                        getattr(state, "vy", 0.0),
                    )
                    if (
                        not all(math.isfinite(float(value)) for value in state_values)
                        or abs(float(state.x)) > 500.0
                        or abs(float(state.y)) > 500.0
                        or abs(float(getattr(state, "vx", 0.0))) > 50.0
                        or abs(float(getattr(state, "vy", 0.0))) > 50.0
                    ):
                        stable = False
                        break
                    distance_travelled += math.hypot(state.x - before_x, state.y - before_y)
                    max_speed = max(max_speed, abs(float(getattr(state, "vx", 0.0))))
                    margin = (
                        float(nearest["width_left"]) - lateral_error
                        if lateral_error >= 0
                        else float(nearest["width_right"]) + lateral_error
                    ) - float(cfg.body_width) / 2.0
                    min_track_margin = min(min_track_margin, margin)
                    completed = lap_distance >= float(track["length"]) * 0.985
                    if step_index % 3 == 0 or completed:
                        points.append({
                            "x": float(state.x), "y": float(state.y),
                            "z": float(getattr(state, "z", 0.0)),
                            "heading": float(getattr(state, "heading", 0.0)),
                            "speed": abs(float(getattr(state, "vx", 0.0))),
                            "time": float((step_index + 1) * cfg.dt),
                        })
                    if completed:
                        break
                path_trajectories.append({
                    "model_id": model_id,
                    "terrain_id": terrain_id,
                    "label": variant["label"] or model_id,
                    "points": points,
                    "distance": distance_travelled,
                    "max_speed": max_speed,
                    "min_track_margin": min_track_margin if math.isfinite(min_track_margin) else None,
                    "completed": stable and lap_distance >= float(track["length"]) * 0.985,
                })
            return {
                "type": "comparison",
                "mode": "f1tenth_path_following",
                "track_id": track_id,
                "physics_model": "planar_path_comparison",
                "terrain": {"id": self.terrain_id, **self.terrain_preset()},
                "terrain_ids": [variant["terrain_id"] for variant in variants],
                "mu": self.effective_mu(),
                "target_speed": speed_ceiling,
                "trajectories": path_trajectories,
            }

        trajectories = []
        for variant in variants:
            model_id = variant["model_id"]
            terrain_id = variant["terrain_id"]
            cfg = self.terrain_config_for(terrain_id, config_with_values(self.cfg, profiles.get(model_id, {})))
            if self.physics_model == "six_dof":
                state: Any = initial_six_dof_state(cfg)
            else:
                state = DynamicBicycleState()
            points = []
            for _ in range(requested_steps):
                if self.physics_model == "six_dof":
                    result = six_dof_step(
                        state, target_speed, steering, self.effective_mu_for(terrain_id), cfg,
                        lambda x, y, terrain_id=terrain_id: self.terrain_height_for(x, y, terrain_id),
                    )
                else:
                    result = dynamic_bicycle_step(
                        state, DynamicCommand(target_speed, steering), self.effective_mu_for(terrain_id), cfg,
                    )
                state = result.state
                points.append({
                    "x": float(state.x), "y": float(state.y),
                    "z": float(getattr(state, "z", 0.0)),
                    "heading": float(getattr(state, "heading", 0.0)),
                })
            trajectories.append({
                "model_id": model_id,
                "terrain_id": terrain_id,
                "label": variant["label"] or model_id,
                "points": points,
            })
        return {
            "type": "comparison",
            "physics_model": self.physics_model,
            "target_speed": target_speed,
            "steering": steering,
            "trajectories": trajectories,
        }

    def handle(self, message: dict[str, Any]) -> dict[str, Any] | None:
        kind = message.get("type")
        if kind in {"reset", "hello"}:
            if kind == "reset":
                self.reset()
            return
        if kind == "play":
            self.playing = bool(message.get("value", True))
            return
        if kind == "control":
            target_speed = float(message.get("target_speed", self.command.speed))
            steering = float(message.get("steering", self.command.steering))
            self.command = DynamicCommand(target_speed, steering)
            return
        if kind == "pose":
            x = float(message.get("x", 0.0))
            y = float(message.get("y", 0.0))
            heading = float(message.get("heading", 0.0))
            self.state.x = x
            self.state.y = y
            self.state.heading = heading
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.yaw_rate = 0.0
            self.state.steering = 0.0
            self.command = DynamicCommand(0.0, 0.0)
            if self.physics_model == "six_dof":
                self.state.roll = 0.0
                self.state.pitch = 0.0
                self.state.vz = 0.0
                self.state.roll_rate = 0.0
                self.state.pitch_rate = 0.0
                self.state.qx = 0.0
                self.state.qy = 0.0
                self.state.qz = math.sin(heading * 0.5)
                self.state.qw = math.cos(heading * 0.5)
                self.state.z = (
                    self.cfg.com_height
                    + self.cfg.wheel_radius
                    + max(0.01, self.cfg.wheel_radius * 0.45)
                    + self.terrain_height(x, y)
                )
            self.sim_time = 0.0
            self.last_step = None
            self.pose_ready = True
            return
        if kind == "compare":
            return self.comparison(message)
        if kind == "parameters":
            next_model = message.get("physics_model", self.physics_model)
            if next_model in {"planar", "six_dof"} and next_model != self.physics_model:
                self.physics_model = next_model
                self.reset()
            if isinstance(message.get("model_id"), str):
                self.vehicle_model = message["model_id"]
            self.update_parameters(message.get("vehicle", {}))
            if "mu" in message:
                self.mu = max(0.01, min(2.0, float(message["mu"])))
            return
        if kind == "terrain":
            terrain_id = str(message.get("terrain_id", self.terrain_id))
            if terrain_id in TERRAIN_PRESETS and terrain_id != self.terrain_id:
                self.terrain_id = terrain_id
                self.reset()
            return
        return None


def websocket_accept(key: str) -> str:
    digest = hashlib.sha1((key + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


async def read_frame(reader: asyncio.StreamReader) -> tuple[int, bytes] | None:
    header = await reader.readexactly(2)
    first, second = header
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", await reader.readexactly(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", await reader.readexactly(8))[0]
    mask = await reader.readexactly(4) if masked else b""
    payload = bytearray(await reader.readexactly(length))
    if masked:
        for index in range(length):
            payload[index] ^= mask[index % 4]
    return opcode, bytes(payload)


def encode_frame(payload: bytes, opcode: int = 1) -> bytes:
    length = len(payload)
    if length < 126:
        header = bytes([0x80 | opcode, length])
    elif length < 65536:
        header = bytes([0x80 | opcode, 126]) + struct.pack("!H", length)
    else:
        header = bytes([0x80 | opcode, 127]) + struct.pack("!Q", length)
    return header + payload


async def websocket_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter, headers: str) -> None:
    header_map = {}
    for line in headers.split("\r\n")[1:]:
        if ":" in line:
            name, value = line.split(":", 1)
            header_map[name.strip().lower()] = value.strip()
    key = header_map.get("sec-websocket-key")
    if not key:
        writer.close()
        await writer.wait_closed()
        return
    response = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {websocket_accept(key)}\r\n\r\n"
    )
    writer.write(response.encode("ascii"))
    await writer.drain()
    session = SimulationSession()
    writer.write(encode_frame(json.dumps(session.snapshot()).encode("utf-8")))
    await writer.drain()

    async def receive() -> None:
        try:
            while True:
                frame = await read_frame(reader)
                if frame is None:
                    return
                opcode, payload = frame
                if opcode == 8:
                    return
                if opcode == 9:
                    writer.write(encode_frame(payload, opcode=10))
                    await writer.drain()
                elif opcode == 1 and payload:
                    try:
                        response = session.handle(json.loads(payload.decode("utf-8")))
                        payload = response if response is not None else session.snapshot()
                        writer.write(encode_frame(json.dumps(payload).encode("utf-8")))
                        await writer.drain()
                    except (ValueError, TypeError, json.JSONDecodeError):
                        continue
        except (asyncio.IncompleteReadError, ConnectionError):
            return

    async def tick() -> None:
        while True:
            await asyncio.sleep(1.0 / 30.0)
            session.advance()
            writer.write(encode_frame(json.dumps(session.snapshot()).encode("utf-8")))
            await writer.drain()

    tasks = [asyncio.create_task(receive()), asyncio.create_task(tick())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for task in tasks:
            task.cancel()
        writer.close()
        try:
            await writer.wait_closed()
        except ConnectionError:
            pass


def safe_file_path(path: str) -> Path | None:
    relative = unquote(urlsplit(path).path).lstrip("/") or "index.html"
    candidate = (VIEWER_ROOT / relative).resolve()
    try:
        candidate.relative_to(VIEWER_ROOT.resolve())
    except ValueError:
        return None
    return candidate if candidate.is_file() else None


async def http_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
    try:
        request = await reader.readuntil(b"\r\n\r\n")
        headers = request.decode("latin1")
        request_line = headers.split("\r\n", 1)[0].split()
        if len(request_line) < 2:
            return
        method, path = request_line[:2]
        upgrade = "upgrade: websocket" in headers.lower()
        if method == "GET" and upgrade and urlsplit(path).path == "/ws":
            await websocket_client(reader, writer, headers)
            return
        if method != "GET":
            body = b"Method Not Allowed"
            response = b"HTTP/1.1 405 Method Not Allowed\r\nContent-Length: 18\r\n\r\n" + body
        else:
            if urlsplit(path).path == "/api/f1tenth/paths":
                body = json.dumps(f1tenth_path_catalog()).encode("utf-8")
                response = (
                    b"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
                    + f"Content-Length: {len(body)}\r\nCache-Control: no-cache\r\n\r\n".encode("ascii")
                    + body
                )
            else:
                file_path = safe_file_path(path)
                if file_path is None:
                    body = b"Not Found"
                    response = b"HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\n\r\n" + body
                else:
                    body = file_path.read_bytes()
                    content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"
                    response = (
                        f"HTTP/1.1 200 OK\r\nContent-Type: {content_type}\r\n"
                        f"Content-Length: {len(body)}\r\nCache-Control: no-cache\r\n\r\n"
                    ).encode("ascii") + body
        writer.write(response)
        await writer.drain()
    except (asyncio.IncompleteReadError, ConnectionError, ValueError):
        pass
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except ConnectionError:
            pass


async def run_server(host: str, port: int) -> None:
    server = await asyncio.start_server(http_client, host, port)
    addresses = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
    print(f"Digital twin viewer: http://{host}:{port} ({addresses})", flush=True)
    async with server:
        await server.serve_forever()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    try:
        asyncio.run(run_server(args.host, args.port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
