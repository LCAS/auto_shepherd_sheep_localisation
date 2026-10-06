"""Small SQLite persistence layer for Boids analysis results."""

from __future__ import annotations

import csv
import io
import json
import os
import sqlite3
import threading
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
        self._connection.commit()

    def save(self, result: dict) -> None:
        with self._lock:
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

    def history(self, limit: int = 120, session_id: Optional[str] = None) -> list[dict]:
        limit = max(1, min(int(limit), 2000))
        query = "SELECT payload FROM boids_analysis"
        params: list = []
        if session_id:
            query += " WHERE session_id = ?"
            params.append(session_id)
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
            "schema_version", "source_type", "session_id", "segment_id", "field_id",
            "scope", "track_id", "window_start_s", "window_end_s", "status",
            "cohesion", "alignment", "separation", "boundary", "active_features",
            "usable_samples", "observed_tracks", "acceleration_rmse_m_s2", "reason_codes",
        ]
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        for result in values:
            coefficients = result.get("coefficients", {})
            quality = result.get("quality", {})
            writer.writerow({
                "schema_version": result.get("schema_version"),
                "source_type": result.get("source_type"),
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
                "boundary": coefficients.get("boundary"),
                "active_features": ",".join(result.get("active_features", [])),
                "usable_samples": quality.get("usable_samples"),
                "observed_tracks": quality.get("observed_tracks"),
                "acceleration_rmse_m_s2": quality.get("acceleration_rmse_m_s2"),
                "reason_codes": ",".join(quality.get("reason_codes", [])),
            })
        return output.getvalue()

    def close(self) -> None:
        with self._lock:
            self._connection.close()
