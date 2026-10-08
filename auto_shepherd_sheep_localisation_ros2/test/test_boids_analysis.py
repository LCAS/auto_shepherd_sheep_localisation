import math

import numpy as np

from auto_shepherd_sheep_localisation_ros2.boids_analysis import (
    BoidsConfig,
    BoidsEstimator,
    RegressionSample,
    boundary_feature,
)


def test_nonnegative_ridge_recovers_independent_features():
    estimator = BoidsEstimator(BoidsConfig(regularisation=1.0e-6))
    rng = np.random.default_rng(7)
    expected = np.array([0.8, 0.35, 0.2])
    samples = []
    for index in range(120):
        features = rng.normal(size=(3, 2))
        target = expected @ features[:3] + rng.normal(0.0, 0.005, size=2)
        samples.append(RegressionSample(float(index), target, features, f"sheep_{index % 5}"))
    weights, diagnostics = estimator._fit(samples, [0, 1, 2])
    assert weights is not None
    assert diagnostics["rank"] == 3
    assert np.allclose(weights, expected, atol=0.04)


def test_three_parameter_mvp_does_not_require_boundary():
    estimator = BoidsEstimator(BoidsConfig(minimum_window_s=0, minimum_samples=1))
    estimator.ingest(0.0, {"a": (0.0, 0.0), "b": (1.0, 0.0)})
    estimator.ingest(1.0, {"a": (0.1, 0.0), "b": (1.1, 0.0)})
    result = estimator.publish_result(force=True)
    assert result["active_features"] == ["cohesion", "alignment", "separation"]
    assert "boundary" not in result["coefficients"]


def test_duplicate_and_clock_rewind_start_clean_segments():
    estimator = BoidsEstimator()
    assert estimator.ingest(1.0, {"a": (0.0, 0.0)})
    assert not estimator.ingest(1.0, {"a": (0.1, 0.0)})
    assert estimator.ingest(0.0, {"a": (0.0, 0.0)})
    assert estimator.segment_id == "segment_002"
    assert estimator.rejection_counts["duplicate_timestamp"] == 1
    assert estimator.rejection_counts["clock_rewind"] == 1


def test_boundary_vector_is_finite_and_outside_is_flagged():
    polygon = [np.array((-10.0, -10.0)), np.array((10.0, -10.0)), np.array((10.0, 10.0)), np.array((-10.0, 10.0))]
    vector, outside, distance = boundary_feature(np.array((9.0, 0.0)), polygon, 5.0, 1.0)
    assert not outside
    assert distance == 1.0
    assert np.all(np.isfinite(vector))
    assert vector[0] < 0.0
    _, outside, _ = boundary_feature(np.array((12.0, 0.0)), polygon, 5.0, 1.0)
    assert outside


def test_persistent_relative_movement_difference_is_reported_as_candidate():
    estimator = BoidsEstimator(
        BoidsConfig(
            minimum_window_s=0,
            minimum_samples=1,
            outlier_min_samples=3,
            outlier_z_threshold=2.0,
            outlier_persistence_s=2.0,
        )
    )
    result = None
    for timestamp in range(8):
        estimator.ingest(
            float(timestamp),
            {
                "stationary": (0.0, 0.0),
                "moving_a": (1.0 + timestamp, 0.0),
                "moving_b": (0.0, 1.0 + timestamp),
                "moving_c": (-1.0 + timestamp, 0.0),
            },
        )
        result = estimator.publish_result(force=True)

    assert result is not None
    assert "stationary" in result["outliers"]
    details = result["per_track"]["stationary"]["movement_outlier"]
    assert details["persistent_flag"] is True
    assert "low_speed_relative_to_flock" in details["reason_codes"]
