from auto_shepherd_sheep_localisation_ros2.boids_storage import BoidsStorage


def test_storage_round_trip_and_csv(tmp_path):
    storage = BoidsStorage(str(tmp_path / "history.sqlite3"))
    result = {
        "schema_version": 1,
        "source_type": "simulation",
        "session_id": "demo",
        "segment_id": "segment_001",
        "field_id": "field_a",
        "scope": "observed_flock",
        "track_id": None,
        "window_start_s": 0.0,
        "window_end_s": 1.0,
        "status": "insufficient_data",
        "coefficients": {"cohesion": None, "alignment": None, "separation": None, "boundary": None},
        "active_features": ["cohesion", "alignment", "separation"],
        "quality": {"usable_samples": 0, "observed_tracks": 0, "acceleration_rmse_m_s2": None, "reason_codes": ["warming_up"]},
        "publication_time_s": 1.0,
    }
    storage.save(result)
    assert storage.latest("demo")["status"] == "insufficient_data"
    csv_text = storage.export_csv(storage.history())
    assert "acceleration_rmse_m_s2" in csv_text
    assert "warming_up" in csv_text
    storage.close()
