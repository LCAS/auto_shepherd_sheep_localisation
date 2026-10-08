"""Small SQLite persistence layer for Boids analysis results."""

from __future__ import annotations

import csv
import io
import json
import os
import sqlite3
import threading
import time
from typing import Iterable, Optional


class BoidsStorage:
    """Durable, append/replace storage keyed by a result window."""

    def __init__(self, path: Optional[str] = None):
        self.path = path or os.environ.get(
            "SHEEP_RADAR_DB", os.path.expanduser("~/.auto_shepherd/boids.sqlite3")
        )
        parent = os.path.dirname(os.path.abspath(self.path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._lock = threading.Lock()
        self._connection = sqlite3.connect(self.path, check_same_thread=False)
        self._connection.row_factory = sqlite3.Row
        self._connection.execute(
            """CREATE TABLE IF NOT EXISTS boids_analysis (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                session_id TEXT NOT NULL,
                segment_id TEXT NOT NULL,
                field_id TEXT NOT NULL,
                scope TEXT NOT NULL,
                track_id TEXT,
                window_start_s REAL NOT NULL,
                window_end_s REAL NOT NULL,
                status TEXT NOT NULL,
                payload TEXT NOT NULL,
                created_at REAL NOT NULL,
                UNIQUE(session_id, segment_id, field_id, scope, track_id,
                       window_start_s, window_end_s)
            )"""
        )
        self._connection.execute(
            "CREATE INDEX IF NOT EXISTS idx_boids_analysis_time "
            "ON boids_analysis(session_id, field_id, window_end_s)"
        )
        self._connection.execute(
            """CREATE TABLE IF NOT EXISTS boids_deleted_sessions (
                session_id TEXT PRIMARY KEY,
                deleted_at REAL NOT NULL
            )"""
        )
        self._connection.commit()

    def is_deleted(self, session_id: Optional[str]) -> bool:
        if not session_id:
            return False
        with self._lock:
            row = self._connection.execute(
                "SELECT 1 FROM boids_deleted_sessions WHERE session_id = ?",
                (session_id,),
            ).fetchone()
        return row is not None

    def save(self, result: dict) -> bool:
        session_id = result.get("session_id", "unknown")
        with self._lock:
            # Check and insert under the same lock so a delete cannot race with
            # a late ROS result and accidentally recreate the bad session.
            deleted = self._connection.execute(
                "SELECT 1 FROM boids_deleted_sessions WHERE session_id = ?",
                (session_id,),
            ).fetchone()
            if deleted is not None:
                return False
            self._connection.execute(
                """INSERT OR REPLACE INTO boids_analysis
                (session_id, segment_id, field_id, scope, track_id,
                 window_start_s, window_end_s, status, payload, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (
                    result.get("session_id", "unknown"),
                    result.get("segment_id", "unknown"),
                    result.get("field_id", "unknown"),
                    result.get("scope", "observed_flock"),
                    result.get("track_id"),
                    float(result.get("window_start_s", 0.0)),
                    float(result.get("window_end_s", 0.0)),
                    result.get("status", "error"),
                    json.dumps(result, allow_nan=False),
                    float(result.get("publication_time_s", 0.0)),
                ),
            )
            self._connection.commit()
        return True

    def delete_session(self, session_id: str) -> int:
        """Delete all stored analysis for a session and tombstone that session."""
        if not session_id or len(session_id) > 512:
            raise ValueError("invalid session_id")
        with self._lock:
            cursor = self._connection.execute(
                "DELETE FROM boids_analysis WHERE session_id = ?", (session_id,)
            )
            self._connection.execute(
                "INSERT OR REPLACE INTO boids_deleted_sessions(session_id, deleted_at) VALUES (?, ?)",
                (session_id, time.time()),
            )
            self._connection.commit()
            return int(cursor.rowcount)

    def history(
        self,
        limit: int = 120,
        session_id: Optional[str] = None,
        field_id: Optional[str] = None,
        segment_id: Optional[str] = None,
    ) -> list[dict]:
        limit = max(1, min(int(limit), 2000))
        query = "SELECT payload FROM boids_analysis"
        params: list = []
        clauses = []
        if session_id:
            clauses.append("session_id = ?")
            params.append(session_id)
        if field_id:
            clauses.append("field_id = ?")
            params.append(field_id)
        if segment_id:
            clauses.append("segment_id = ?")
            params.append(segment_id)
        if clauses:
            query += " WHERE " + " AND ".join(clauses)
        query += " ORDER BY window_end_s DESC, id DESC LIMIT ?"
        params.append(limit)
        with self._lock:
            rows = self._connection.execute(query, params).fetchall()
        values = [json.loads(row["payload"]) for row in rows]
        values.reverse()
        return values

    def latest(self, session_id: Optional[str] = None) -> Optional[dict]:
        values = self.history(limit=1, session_id=session_id)
        return values[-1] if values else None

    def export_csv(self, values: Iterable[dict]) -> str:
        output = io.StringIO()
        fieldnames = [
            "schema_version", "source_type", "detector_model_id", "session_id", "segment_id", "field_id",
            "scope", "track_id", "window_start_s", "window_end_s", "status",
            "cohesion", "alignment", "separation", "active_features",
            "usable_samples", "observed_tracks", "acceleration_rmse_m_s2", "reason_codes",
            "candidate_outlier_count", "candidate_outlier_ids", "outlier_analysis_status",
            "flock_cohesion_01", "flock_alignment_01", "flock_separation_01",
        ]
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for result in values:
            coefficients = result.get("coefficients", {})
            quality = result.get("quality", {})
            writer.writerow({
                "schema_version": result.get("schema_version"),
                "source_type": result.get("source_type"),
                "detector_model_id": result.get("detector_model_id"),
                "session_id": result.get("session_id"),
                "segment_id": result.get("segment_id"),
                "field_id": result.get("field_id"),
                "scope": result.get("scope"),
                "track_id": result.get("track_id"),
                "window_start_s": result.get("window_start_s"),
                "window_end_s": result.get("window_end_s"),
                "status": result.get("status"),
                "cohesion": coefficients.get("cohesion"),
                "alignment": coefficients.get("alignment"),
                "separation": coefficients.get("separation"),
                "active_features": ",".join(result.get("active_features", [])),
                "usable_samples": quality.get("usable_samples"),
                "observed_tracks": quality.get("observed_tracks"),
                "acceleration_rmse_m_s2": quality.get("acceleration_rmse_m_s2"),
                "reason_codes": ",".join(quality.get("reason_codes", [])),
                "candidate_outlier_count": len(result.get("outliers", [])),
                "candidate_outlier_ids": ",".join(str(track_id) for track_id in result.get("outliers", [])),
                "outlier_analysis_status": result.get("outlier_analysis", {}).get("status"),
                "flock_cohesion_01": result.get("flock_indicators_01", {}).get("cohesion"),
                "flock_alignment_01": result.get("flock_indicators_01", {}).get("alignment"),
                "flock_separation_01": result.get("flock_indicators_01", {}).get("separation"),
            })
        return output.getvalue()

    def export_track_csv(self, values: Iterable[dict]) -> str:
        """Export one row per tracked sheep and analysis window."""
        output = io.StringIO()
        fieldnames = [
            "schema_version", "source_type", "detector_model_id", "session_id", "segment_id",
            "field_id", "window_start_s", "window_end_s", "track_id", "track_status",
            "cohesion_gain", "alignment_gain", "separation_gain", "mean_speed_m_s",
            "stopping_fraction", "distance_from_flock_m", "cohesion_01", "alignment_01",
            "separation_01", "candidate_flag", "persistent_flag", "persistence_duration_s",
            "reason_codes",
        ]
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for result in values:
            for track_id, track in (result.get("per_track") or {}).items():
                outlier = track.get("movement_outlier") or {}
                indicators = outlier.get("normalised_indicators") or {}
                coefficients = track.get("coefficients") or {}
                writer.writerow({
                    "schema_version": result.get("schema_version"),
                    "source_type": result.get("source_type"),
                    "detector_model_id": result.get("detector_model_id"),
                    "session_id": result.get("session_id"),
                    "segment_id": result.get("segment_id"),
                    "field_id": result.get("field_id"),
                    "window_start_s": result.get("window_start_s"),
                    "window_end_s": result.get("window_end_s"),
                    "track_id": track.get("track_id", track_id),
                    "track_status": track.get("status"),
                    "cohesion_gain": coefficients.get("cohesion"),
                    "alignment_gain": coefficients.get("alignment"),
                    "separation_gain": coefficients.get("separation"),
                    "mean_speed_m_s": outlier.get("mean_speed_m_s"),
                    "stopping_fraction": outlier.get("stopping_fraction"),
                    "distance_from_flock_m": outlier.get("distance_from_flock_m"),
                    "cohesion_01": indicators.get("cohesion"),
                    "alignment_01": indicators.get("alignment"),
                    "separation_01": indicators.get("separation"),
                    "candidate_flag": outlier.get("candidate_flag"),
                    "persistent_flag": outlier.get("persistent_flag"),
                    "persistence_duration_s": outlier.get("persistence_duration_s"),
                    "reason_codes": ",".join(outlier.get("reason_codes", [])),
                })
        return output.getvalue()

    def close(self) -> None:
        with self._lock:
            self._connection.close()
