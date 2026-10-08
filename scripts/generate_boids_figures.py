#!/usr/bin/env python3
"""Generate reproducible publication figures from a Boids CSV export.

Example:
    python scripts/generate_boids_figures.py \
        --input boids_analysis.csv \
        --output-dir paper_figures

HTML figures are always written. SVG/PNG files are written when the optional
Kaleido Plotly renderer is installed.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


FEATURES = (
    ("cohesion", "Cohesion", "#2563eb"),
    ("alignment", "Alignment", "#16a34a"),
    ("separation", "Separation", "#dc2626"),
)


def number(row: dict, key: str):
    value = row.get(key, "")
    try:
        return float(value) if value not in (None, "") else None
    except (TypeError, ValueError):
        return None


def read_rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_figure(figure, output_dir: Path, stem: str) -> None:
    figure.write_html(output_dir / f"{stem}.html", include_plotlyjs="cdn")
    for extension in ("svg", "png"):
        try:
            figure.write_image(output_dir / f"{stem}.{extension}")
        except Exception as exc:  # Kaleido is optional for local paper export.
            print(f"Skipping {stem}.{extension}: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="CSV from /boids/export.csv")
    parser.add_argument("--output-dir", type=Path, default=Path("paper_figures"))
    args = parser.parse_args()

    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as exc:
        raise SystemExit(
            "Plotly is required. Install auto_shepherd_sheep_localisation_ros2/requirements.txt first."
        ) from exc

    rows = read_rows(args.input)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    x = [number(row, "window_end_s") for row in rows]

    coefficient_figure = go.Figure()
    for key, label, colour in FEATURES:
        coefficient_figure.add_trace(go.Scatter(
            x=x,
            y=[number(row, key) for row in rows],
            mode="lines",
            name=label,
            connectgaps=False,
            line={"color": colour, "width": 2},
        ))
    coefficient_figure.update_layout(
        title="Rolling flocking influence estimates",
        xaxis_title="Source time (s)",
        yaxis_title="Fitted gain",
        template="plotly_white",
        hovermode="x unified",
    )
    write_figure(coefficient_figure, args.output_dir, "boids_coefficients")

    quality_figure = make_subplots(specs=[[{"secondary_y": True}]])
    quality_figure.add_trace(go.Scatter(
        x=x,
        y=[number(row, "observed_tracks") for row in rows],
        mode="lines",
        name="Observed sheep",
        line={"color": "#0f766e", "width": 2},
    ), secondary_y=False)
    quality_figure.add_trace(go.Scatter(
        x=x,
        y=[number(row, "acceleration_rmse_m_s2") for row in rows],
        mode="lines",
        name="Acceleration RMSE",
        line={"color": "#d97706", "width": 2, "dash": "dot"},
    ), secondary_y=True)
    quality_figure.update_layout(
        title="Observation and fit quality",
        xaxis_title="Source time (s)",
        template="plotly_white",
        hovermode="x unified",
    )
    quality_figure.update_yaxes(title_text="Observed sheep", secondary_y=False)
    quality_figure.update_yaxes(title_text="RMSE (m/s²)", secondary_y=True)
    write_figure(quality_figure, args.output_dir, "boids_quality")
    print(f"Wrote figures to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
