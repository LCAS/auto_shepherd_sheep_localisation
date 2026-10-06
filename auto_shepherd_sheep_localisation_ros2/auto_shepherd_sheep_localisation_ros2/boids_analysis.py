"""Pure, bounded Boids influence estimation.

The ROS adapter supplies positions in a local metric frame.  This module does
not know about ROS, GPS, or the browser, which keeps the numerical contract
usable from tests, replays, and synthetic fixtures.

The four feature vectors are deliberately defined as acceleration-like
vectors (m/s^2) before fitting.  Coefficients are therefore dimensionless
model gains, not percentages:

* cohesion: normalized vector to the mean neighbour position;
* alignment: normalized difference between neighbour and subject velocity;
* separation: repulsion weighted by distance inside the separation radius;
* boundary: normalized vector away from the nearest polygon edge.

Fixed configured scales make the feature definitions comparable across
windows.  They are not re-normalized during fitting.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
import math
import time
from typing import Deque, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


FEATURES = ("cohesion", "alignment", "separation", "boundary")


@dataclass
class BoidsConfig:
    fit_window_s: float = 60.0
    publish_interval_s: float = 1.0
    minimum_window_s: float = 30.0
    minimum_samples: int = 60
    minimum_track_samples: int = 30
    maximum_gap_s: float = 2.0
    neighbour_radius_m: float = 15.0
    separation_radius_m: float = 5.0
    boundary_radius_m: float = 15.0
    cohesion_accel_scale: float = 0.5
    alignment_accel_scale: float = 0.5
    separation_accel_scale: float = 1.0
    boundary_accel_scale: float = 0.5
    regularisation: float = 0.01
    condition_limit: float = 1.0e6
    stale_after_s: float = 5.0
    max_history_s: float = 180.0


@dataclass
class MotionPoint:
    timestamp: float
    position: np.ndarray
    velocity: Optional[np.ndarray] = None
    acceleration: Optional[np.ndarray] = None


@dataclass
class RegressionSample:
    timestamp: float
    target: np.ndarray
    features: np.ndarray  # shape (4, 2)
    track_id: str


@dataclass
class _Track:
    points: Deque[MotionPoint] = field(default_factory=deque)


def _finite_vector(value: Sequence[float]) -> Optional[np.ndarray]:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (2,) or not np.all(np.isfinite(vector)):
        return None
    return vector


def _nearest_point_on_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, float]:
    edge = b - a
    length_sq = float(np.dot(edge, edge))
    if length_sq <= 1.0e-12:
        nearest = a.copy()
    else:
        t = float(np.dot(point - a, edge) / length_sq)
        nearest = a + max(0.0, min(1.0, t)) * edge
    delta = point - nearest
    return nearest, float(np.linalg.norm(delta))


def point_in_polygon(point: np.ndarray, polygon: Sequence[np.ndarray]) -> bool:
    """Return whether point is inside a simple polygon using ray casting."""
    inside = False
    if len(polygon) < 3:
        return False
    j = len(polygon) - 1
    for i, current in enumerate(polygon):
        previous = polygon[j]
        crosses = ((current[1] > point[1]) != (previous[1] > point[1]))
        if crosses:
            x_at_y = (previous[0] - current[0]) * (point[1] - current[1]) / (
                previous[1] - current[1] + 1.0e-15
            ) + current[0]
            if point[0] < x_at_y:
                inside = not inside
        j = i
    return inside


def boundary_feature(
    point: np.ndarray,
    polygon: Optional[Sequence[np.ndarray]],
    radius_m: float,
    scale: float,
) -> Tuple[np.ndarray, bool, Optional[float]]:
    """Return an away-from-edge vector, out-of-bound flag, and edge distance."""
    if not polygon or len(polygon) < 3:
        return np.zeros(2), False, None
    nearest = None
    distance = math.inf
    for index, start in enumerate(polygon):
        candidate, candidate_distance = _nearest_point_on_segment(
            point, start, polygon[(index + 1) % len(polygon)]
        )
        if candidate_distance < distance:
            nearest, distance = candidate, candidate_distance
    inside = point_in_polygon(point, polygon)
    if not inside:
        return np.zeros(2), True, distance
    if distance <= 1.0e-9 or distance >= radius_m:
        return np.zeros(2), False, distance
    away = point - nearest
    norm = float(np.linalg.norm(away))
    if norm <= 1.0e-9:
        return np.zeros(2), False, distance
    return (away / norm) * scale * (radius_m - distance) / radius_m, False, distance


class BoidsEstimator:
    """Causal rolling estimator for a single source/session segment."""

    def __init__(self, config: Optional[BoidsConfig] = None):
        self.config = config or BoidsConfig()
        self.tracks: Dict[str, _Track] = defaultdict(_Track)
        self.samples: Deque[RegressionSample] = deque()
        self.track_samples: Dict[str, Deque[RegressionSample]] = defaultdict(deque)
        self.latest_features: Dict[str, np.ndarray] = {}
        self.latest_positions: Dict[str, np.ndarray] = {}
        self.boundary: Optional[List[np.ndarray]] = None
        self.boundary_revision: Optional[str] = None
        self.latest_timestamp: Optional[float] = None
        self.last_batch_timestamp: Optional[float] = None
        self.last_publish_timestamp: Optional[float] = None
        self.segment_number = 1
        self.rejection_counts: Dict[str, int] = defaultdict(int)
        self.last_reasons: List[str] = []
        self.out_of_bound_count = 0
        self.source_session = "session_001"
        self.source_type = "replay"
        self.field_id = "field_default"
        self._last_result: Optional[dict] = None

    @property
    def segment_id(self) -> str:
        return f"segment_{self.segment_number:03d}"

    def configure_identity(self, session_id: str, field_id: str, source_type: str) -> None:
        self.source_session = session_id
        self.field_id = field_id
        self.source_type = source_type

    def reset_segment(self, reason: str = "replay_reset") -> None:
        """Discard motion history when a source starts a new replay segment."""
        self._clear_segment(reason)

    def set_boundary(self, points: Optional[Iterable[Sequence[float]]], revision: Optional[str] = None) -> None:
        if points is None:
            self.boundary = None
            self.boundary_revision = None
            return
        converted = [_finite_vector(point) for point in points]
        converted = [point for point in converted if point is not None]
        self.boundary = converted if len(converted) >= 3 else None
        self.boundary_revision = revision or (f"boundary_{len(converted)}" if self.boundary else None)

    def _clear_segment(self, reason: str) -> None:
        self.tracks.clear()
        self.samples.clear()
        self.track_samples.clear()
        self.latest_features.clear()
        self.latest_positions.clear()
        self.latest_timestamp = None
        self.last_batch_timestamp = None
        self.out_of_bound_count = 0
        self.last_publish_timestamp = None
        self.segment_number += 1
        self.rejection_counts[reason] += 1
        self.last_reasons = [reason]

    def ingest(self, timestamp: float, positions: Mapping[str, Sequence[float]]) -> bool:
        """Consume one timestamped visible-track snapshot.

        Returns False for duplicate/non-increasing snapshots.  A source clock
        rewind starts a new segment so old velocity and acceleration cannot be
        mixed with the new replay interval.
        """
        if not math.isfinite(timestamp):
            self.rejection_counts["invalid_timestamp"] += 1
            return False
        if self.latest_timestamp is not None and timestamp < self.latest_timestamp - 1.0e-6:
            self._clear_segment("clock_rewind")
        if self.last_batch_timestamp is not None and timestamp <= self.last_batch_timestamp + 1.0e-6:
            self.rejection_counts["duplicate_timestamp"] += 1
            return False

        self.last_reasons = []
        current: Dict[str, np.ndarray] = {}
        for track_id, raw_position in positions.items():
            position = _finite_vector(raw_position)
            if position is None:
                self.rejection_counts["invalid_position"] += 1
                continue
            current[str(track_id)] = position

        self.latest_positions = current
        self.latest_timestamp = timestamp
        self.last_batch_timestamp = timestamp
        current_motion: Dict[str, MotionPoint] = {}
        for track_id, position in current.items():
            track = self.tracks[track_id]
            previous = track.points[-1] if track.points else None
            point = MotionPoint(timestamp=timestamp, position=position)
            if previous is not None:
                dt = timestamp - previous.timestamp
                if dt <= 0.0:
                    self.rejection_counts["non_increasing_track_time"] += 1
                    continue
                if dt > self.config.maximum_gap_s:
                    self.rejection_counts["track_gap"] += 1
                    track.points.clear()
                else:
                    point.velocity = (position - previous.position) / dt
                    if previous.velocity is not None and len(track.points) >= 2:
                        velocity_dt = timestamp - track.points[-2].timestamp
                        if velocity_dt > 0.0 and velocity_dt <= self.config.maximum_gap_s:
                            point.acceleration = (point.velocity - previous.velocity) / velocity_dt
            track.points.append(point)
            current_motion[track_id] = point
            cutoff = timestamp - self.config.max_history_s
            while track.points and track.points[0].timestamp < cutoff:
                track.points.popleft()

        self.latest_features = {}
        visible = list(current_motion.items())
        for track_id, point in visible:
            if point.velocity is None:
                continue
            neighbours = [
                (other_id, other_point)
                for other_id, other_point in visible
                if other_id != track_id and other_point.velocity is not None
            ]
            features = self._features(point, neighbours)
            self.latest_features[track_id] = features
            if point.acceleration is None:
                continue
            sample = RegressionSample(timestamp, point.acceleration, features, track_id)
            self.samples.append(sample)
            self.track_samples[track_id].append(sample)

        self._prune_samples(timestamp)
        return True

    def _features(self, point: MotionPoint, neighbours: Sequence[Tuple[str, MotionPoint]]) -> np.ndarray:
        cfg = self.config
        cohesion = np.zeros(2)
        alignment = np.zeros(2)
        separation = np.zeros(2)
        neighbour_count = 0
        alignment_count = 0
        for _, neighbour in neighbours:
            delta = neighbour.position - point.position
            distance = float(np.linalg.norm(delta))
            if distance <= 1.0e-9 or distance > cfg.neighbour_radius_m:
                continue
            neighbour_count += 1
            cohesion += delta / cfg.neighbour_radius_m
            alignment += (neighbour.velocity - point.velocity) / max(cfg.neighbour_radius_m, 1.0)
            alignment_count += 1
            if distance < cfg.separation_radius_m:
                separation -= (delta / distance) * (cfg.separation_radius_m - distance) / cfg.separation_radius_m
        if neighbour_count:
            cohesion = cohesion / neighbour_count * cfg.cohesion_accel_scale
        if alignment_count:
            alignment = alignment / alignment_count * cfg.alignment_accel_scale
        if neighbour_count:
            separation = separation / neighbour_count * cfg.separation_accel_scale
        boundary, outside, _ = boundary_feature(
            point.position,
            self.boundary,
            cfg.boundary_radius_m,
            cfg.boundary_accel_scale,
        )
        if outside:
            self.out_of_bound_count += 1
        return np.vstack((cohesion, alignment, separation, boundary))

    def _prune_samples(self, timestamp: float) -> None:
        cutoff = timestamp - self.config.fit_window_s
        while self.samples and self.samples[0].timestamp < cutoff:
            self.samples.popleft()
        for track_id, samples in list(self.track_samples.items()):
            while samples and samples[0].timestamp < cutoff:
                samples.popleft()
            if not samples:
                del self.track_samples[track_id]

    def should_publish(self) -> bool:
        if self.latest_timestamp is None:
            return False
        return (
            self.last_publish_timestamp is None
            or self.latest_timestamp - self.last_publish_timestamp >= self.config.publish_interval_s
        )

    def publish_result(self, force: bool = False, wall_timestamp: Optional[float] = None) -> Optional[dict]:
        if self.latest_timestamp is None:
            return None
        if not force and not self.should_publish():
            return self._last_result
        self.last_publish_timestamp = self.latest_timestamp
        result = self._make_result(wall_timestamp=wall_timestamp)
        self._last_result = result
        return result

    def _fit(self, samples: Sequence[RegressionSample], active_indices: Sequence[int]) -> Tuple[Optional[np.ndarray], dict]:
        if not samples:
            return None, {"reason_codes": ["no_usable_samples"]}
        rows = []
        targets = []
        for sample in samples:
            rows.append(sample.features[:, 0][list(active_indices)])
            rows.append(sample.features[:, 1][list(active_indices)])
            targets.extend([sample.target[0], sample.target[1]])
        matrix = np.asarray(rows, dtype=float)
        target = np.asarray(targets, dtype=float)
        if matrix.shape[0] < len(active_indices) * 2:
            return None, {"reason_codes": ["insufficient_samples"]}
        singular = np.linalg.svd(matrix, compute_uv=False)
        rank = int(np.linalg.matrix_rank(matrix))
        condition = float(singular[0] / singular[-1]) if singular.size and singular[-1] > 1.0e-12 else math.inf
        diagnostics = {"rank": rank, "condition_number": condition}
        if rank < len(active_indices):
            return None, {**diagnostics, "reason_codes": ["rank_deficient"]}
        if condition > self.config.condition_limit:
            return None, {**diagnostics, "reason_codes": ["ill_conditioned"]}

        count = max(1, len(samples))
        gram = (matrix.T @ matrix) / count + self.config.regularisation * np.eye(len(active_indices))
        rhs = (matrix.T @ target) / count
        # Projected-gradient non-negative ridge fit. This avoids an optional
        # solver dependency while preserving the requested w >= 0 contract.
        try:
            lipschitz = float(np.max(np.linalg.eigvalsh(gram)))
        except np.linalg.LinAlgError:
            return None, {**diagnostics, "reason_codes": ["solver_error"]}
        if not math.isfinite(lipschitz) or lipschitz <= 0.0:
            return None, {**diagnostics, "reason_codes": ["solver_error"]}
        weights = np.zeros(len(active_indices), dtype=float)
        for _ in range(4000):
            updated = np.maximum(0.0, weights - (gram @ weights - rhs) / lipschitz)
            if float(np.max(np.abs(updated - weights))) < 1.0e-8:
                weights = updated
                break
            weights = updated
        residual = target - matrix @ weights
        diagnostics.update({
            "acceleration_rmse_m_s2": float(math.sqrt(float(np.mean(residual * residual)))),
            "solver": "projected_gradient_nonnegative_ridge",
            "iterations": 4000,
            "regularisation": self.config.regularisation,
            "bounded_nonnegative": True,
            "bounded_features": [
                FEATURES[index] for index, weight in zip(active_indices, weights) if weight <= 1.0e-8
            ],
        })
        return weights, diagnostics

    def _make_result(self, wall_timestamp: Optional[float] = None) -> dict:
        timestamp = self.latest_timestamp or 0.0
        wall_timestamp = wall_timestamp if wall_timestamp is not None else time.time()
        cutoff = timestamp - self.config.fit_window_s
        samples = [sample for sample in self.samples if sample.timestamp >= cutoff]
        active_indices = [0, 1, 2] + ([3] if self.boundary else [])
        active_features = [FEATURES[index] for index in active_indices]
        coefficients = {name: None for name in FEATURES}
        reasons: List[str] = []
        duration = (timestamp - samples[0].timestamp) if samples else 0.0
        status = "warming_up"
        quality = {
            "reason_codes": reasons,
            "usable_samples": len(samples),
            "observed_tracks": len({sample.track_id for sample in samples}),
            "acceleration_rmse_m_s2": None,
            "window_duration_s": duration,
            "rejections": dict(self.rejection_counts),
            "diagnostics": {},
            "boundary_available": bool(self.boundary),
            "out_of_bound_positions": self.out_of_bound_count,
        }
        if not self.boundary:
            reasons.append("boundary_unavailable")
        if self.out_of_bound_count:
            reasons.append("out_of_bound_position")
        if not samples:
            status = "insufficient_data"
            reasons.append("no_usable_samples")
        elif duration < self.config.minimum_window_s or len(samples) < self.config.minimum_samples:
            reasons.append("warming_up")
        else:
            weights, diagnostics = self._fit(samples, active_indices)
            quality["diagnostics"] = diagnostics
            if weights is None:
                status = "insufficient_data"
                reasons.extend(diagnostics.get("reason_codes", ["fit_unavailable"]))
            else:
                status = "ok"
                for index, weight in zip(active_indices, weights):
                    coefficients[FEATURES[index]] = float(weight)
                quality["acceleration_rmse_m_s2"] = diagnostics.get("acceleration_rmse_m_s2")
                if quality["acceleration_rmse_m_s2"] is not None and quality["acceleration_rmse_m_s2"] > 2.0:
                    status = "low_quality"
                    reasons.append("high_acceleration_residual")
        if self.latest_timestamp is not None and timestamp - self.latest_timestamp > self.config.stale_after_s:
            status = "stale"
            reasons.append("source_stale")
        quality["reason_codes"] = sorted(set(reasons))

        per_track = {}
        for track_id, track_samples in self.track_samples.items():
            usable = [sample for sample in track_samples if sample.timestamp >= cutoff]
            if len(usable) < self.config.minimum_track_samples:
                continue
            track_duration = usable[-1].timestamp - usable[0].timestamp
            if track_duration < self.config.minimum_window_s:
                continue
            weights, _ = self._fit(usable, active_indices)
            if weights is None:
                continue
            track_coefficients = {name: None for name in FEATURES}
            for index, weight in zip(active_indices, weights):
                track_coefficients[FEATURES[index]] = float(weight)
            per_track[track_id] = {
                "track_id": track_id,
                "coefficients": track_coefficients,
                "status": "ok",
                "usable_samples": len(usable),
                "window_duration_s": track_duration,
            }

        vectors = {}
        for track_id, features in self.latest_features.items():
            vectors[track_id] = {
                name: {"x_m_s2": float(features[index, 0]), "y_m_s2": float(features[index, 1])}
                for index, name in enumerate(FEATURES)
            }
        return {
            "schema_version": 1,
            "source_type": self.source_type,
            "session_id": self.source_session,
            "segment_id": self.segment_id,
            "field_id": self.field_id,
            "scope": "observed_flock",
            "track_id": None,
            "window_start_s": max(0.0, timestamp - self.config.fit_window_s),
            "window_end_s": timestamp,
            "publication_time_s": wall_timestamp,
            "clock_domain": "ros_source_time",
            "units": {
                "coefficients": "dimensionless gain; fitted against acceleration-like vectors in m/s^2",
                "influence_vectors": "m/s^2",
                "rmse": "m/s^2",
            },
            "active_features": active_features,
            "coefficients": coefficients,
            "status": status,
            "quality": quality,
            "model": {
                "name": "bounded_rolling_boids_ridge",
                "vector_version": "boids-v1",
                "configuration": self.config.__dict__.copy(),
                "boundary_revision": self.boundary_revision,
            },
            "per_track": per_track,
            "vectors": vectors,
        }
