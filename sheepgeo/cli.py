"""
Command-line interface for sheepgeo.

Process drone videos to detect sheep and convert pixel coordinates to GPS.
"""
import argparse
import logging
import json
import csv
from pathlib import Path
from typing import List, Dict, Any
from datetime import datetime

import cv2
from rich.logging import RichHandler
from rich.progress import Progress, SpinnerColumn, BarColumn, TextColumn, TimeRemainingColumn

from sheepgeo.config import (
    DEFAULT_CONFIDENCE, DEFAULT_IOU_THRESHOLD,
    DEFAULT_AGL_OFFSET_M, DEFAULT_GROUND_ELEV_M,
    DEFAULT_OUTPUT_DIR, DEFAULT_GEOJSON_NAME, DEFAULT_CSV_NAME
)
from sheepgeo.io.srt_reader import SrtReader, apply_fallback_defaults
from sheepgeo.yolo.detect import SheepDetector, filter_detections_by_class
from sheepgeo.geo.camera_models import get_camera_config
from sheepgeo.geo.projection import project_detection_bbox_center


# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)]
)

logger = logging.getLogger(__name__)


def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Detect sheep in drone videos and geo-reference their positions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Input files
    parser.add_argument(
        "--video",
        type=Path,
        required=True,
        help="Path to input video file (e.g., DJI_0001.MP4)"
    )
    
    parser.add_argument(
        "--srt",
        type=Path,
        required=True,
        help="Path to DJI SRT subtitle file (e.g., DJI_0001.SRT)"
    )
    
    # Drone configuration
    parser.add_argument(
        "--drone",
        type=str,
        choices=["mavic_air_2", "mavic_air_2_1080p", "mini_2_se", "mini_2_se_1080p"],
        required=True,
        help="Drone model (determines camera parameters)"
    )
    
    # YOLO configuration
    parser.add_argument(
        "--weights",
        type=Path,
        default="yolov8n.pt",
        help="Path to YOLO weights file"
    )
    
    parser.add_argument(
        "--conf",
        type=float,
        default=DEFAULT_CONFIDENCE,
        help="Confidence threshold for detections"
    )
    
    parser.add_argument(
        "--iou",
        type=float,
        default=DEFAULT_IOU_THRESHOLD,
        help="IoU threshold for NMS"
    )
    
    parser.add_argument(
        "--track",
        action="store_true",
        help="Enable object tracking (ByteTrack)"
    )
    
    parser.add_argument(
        "--filter-class",
        type=str,
        nargs="+",
        default=["sheep"],
        help="Filter detections to specific class names"
    )
    
    # Projection configuration
    parser.add_argument(
        "--agl-offset",
        type=float,
        default=DEFAULT_AGL_OFFSET_M,
        help="Assumed height above ground (AGL offset) in meters"
    )
    
    parser.add_argument(
        "--ground-elev",
        type=float,
        default=DEFAULT_GROUND_ELEV_M,
        help="Ground elevation in meters (default: 0 = sea level)"
    )
    
    # Camera overrides
    parser.add_argument(
        "--sensor-fov-deg",
        type=float,
        help="Override sensor FOV in degrees"
    )
    
    parser.add_argument(
        "--focal-px",
        type=float,
        help="Override focal length in pixels"
    )
    
    parser.add_argument(
        "--calib",
        type=Path,
        help="Path to camera calibration file (YAML/JSON)"
    )
    
    # Output configuration
    parser.add_argument(
        "--out",
        type=Path,
        help="Output GeoJSON file path (default: outputs/sheep_positions.geojson)"
    )
    
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(DEFAULT_OUTPUT_DIR),
        help="Output directory"
    )
    
    # Processing options
    parser.add_argument(
        "--max-frames",
        type=int,
        help="Maximum number of frames to process (for testing)"
    )
    
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Generate visualization (HTML map with Folium)"
    )
    
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Enable debug logging"
    )
    
    return parser.parse_args()


def process_video(args) -> List[Dict[str, Any]]:
    """
    Main processing pipeline: detect sheep and geo-reference them.
    
    Args:
        args: Parsed command-line arguments
        
    Returns:
        List of detection results with GPS coordinates
    """
    # Set log level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Load SRT telemetry
    logger.info(f"Loading telemetry from {args.srt}")
    srt_reader = SrtReader(args.srt)
    telemetry_list = srt_reader.parse_all()
    logger.info(f"Loaded telemetry for {len(telemetry_list)} frames")
    
    # Get camera configuration
    logger.info(f"Loading camera config for {args.drone}")
    
    # Get video dimensions
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {args.video}")
    
    video_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    video_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    video_fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    
    logger.info(f"Video: {video_width}x{video_height} @ {video_fps:.2f} FPS, {total_frames} frames")
    
    camera_config = get_camera_config(
        args.drone,
        image_width_px=video_width,
        image_height_px=video_height
    )
    
    # Override camera parameters if provided
    if args.sensor_fov_deg is not None:
        camera_config.fov_deg = args.sensor_fov_deg
        logger.info(f"Override FOV: {args.sensor_fov_deg}°")
    
    # Initialize detector
    logger.info(f"Initializing YOLO detector: {args.weights}")
    detector = SheepDetector(
        weights_path=str(args.weights),
        confidence=args.conf,
        iou_threshold=args.iou,
        track=args.track
    )
    
    # Process video with progress bar
    logger.info("Processing video...")
    
    all_results = []
    
    cap = cv2.VideoCapture(str(args.video))
    frame_index = 0
    
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        BarColumn(),
        TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        TimeRemainingColumn(),
    ) as progress:
        
        task = progress.add_task(
            f"Processing video...",
            total=min(args.max_frames, total_frames) if args.max_frames else total_frames
        )
        
        while True:
            ret, frame = cap.read()
            
            if not ret:
                break
            
            # Run detection
            detections = detector.detect_frame(frame, frame_index)
            
            # Filter to sheep class if requested
            if args.filter_class:
                detections = filter_detections_by_class(
                    detections,
                    args.filter_class,
                    detector.model
                )
            
            # Get telemetry for this frame
            if frame_index < len(telemetry_list):
                telemetry = telemetry_list[frame_index]
                telemetry = apply_fallback_defaults(telemetry)
            else:
                logger.warning(f"Frame {frame_index}: No telemetry available")
                telemetry = None
            
            # Project each detection to GPS
            for detection in detections:
                if telemetry is None:
                    continue
                
                # Project bbox center to GPS
                projection = project_detection_bbox_center(
                    detection['bbox_xyxy'],
                    camera_config,
                    telemetry,
                    agl_offset_m=args.agl_offset,
                    ground_elev_m=args.ground_elev
                )
                
                if projection is None:
                    continue
                
                center_u, center_v, lat, lon, alt_est = projection
                
                # Build result record
                result = {
                    'frame_index': frame_index,
                    'timestamp': str(telemetry.timestamp),
                    'confidence': detection['confidence'],
                    'bbox_xyxy': detection['bbox_xyxy'],
                    'image_xy': [center_u, center_v],
                    'lat': lat,
                    'lon': lon,
                    'alt_est': alt_est,
                    'source_video': str(args.video.name),
                    'drone_model': args.drone,
                    'track_id': detection['track_id']
                }
                
                all_results.append(result)
            
            frame_index += 1
            progress.update(task, advance=1)
            
            # Check max frames
            if args.max_frames and frame_index >= args.max_frames:
                break
    
    cap.release()
    
    logger.info(f"Processed {frame_index} frames, {len(all_results)} sheep detections")
    
    return all_results


def save_geojson(results: List[Dict[str, Any]], output_path: Path):
    """
    Save results as GeoJSON FeatureCollection.
    
    Args:
        results: List of detection results
        output_path: Output file path
    """
    features = []
    
    for result in results:
        feature = {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [result['lon'], result['lat'], result['alt_est']]
            },
            "properties": {
                "frame_index": result['frame_index'],
                "timestamp": result['timestamp'],
                "confidence": result['confidence'],
                "bbox_xyxy": result['bbox_xyxy'],
                "image_xy": result['image_xy'],
                "source_video": result['source_video'],
                "drone_model": result['drone_model'],
                "track_id": result['track_id']
            }
        }
        features.append(feature)
    
    geojson = {
        "type": "FeatureCollection",
        "features": features,
        "properties": {
            "generated_at": datetime.utcnow().isoformat(),
            "total_detections": len(results)
        }
    }
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w') as f:
        json.dump(geojson, f, indent=2)
    
    logger.info(f"Saved GeoJSON: {output_path}")


def save_csv(results: List[Dict[str, Any]], output_path: Path):
    """
    Save results as CSV.
    
    Args:
        results: List of detection results
        output_path: Output file path
    """
    if not results:
        logger.warning("No results to save")
        return
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    fieldnames = [
        'frame_index', 'timestamp', 'lat', 'lon', 'alt_est',
        'confidence', 'image_x', 'image_y',
        'bbox_x1', 'bbox_y1', 'bbox_x2', 'bbox_y2',
        'track_id', 'source_video', 'drone_model'
    ]
    
    with open(output_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        
        for result in results:
            row = {
                'frame_index': result['frame_index'],
                'timestamp': result['timestamp'],
                'lat': result['lat'],
                'lon': result['lon'],
                'alt_est': result['alt_est'],
                'confidence': result['confidence'],
                'image_x': result['image_xy'][0],
                'image_y': result['image_xy'][1],
                'bbox_x1': result['bbox_xyxy'][0],
                'bbox_y1': result['bbox_xyxy'][1],
                'bbox_x2': result['bbox_xyxy'][2],
                'bbox_y2': result['bbox_xyxy'][3],
                'track_id': result['track_id'],
                'source_video': result['source_video'],
                'drone_model': result['drone_model']
            }
            writer.writerow(row)
    
    logger.info(f"Saved CSV: {output_path}")


def create_visualization(results: List[Dict[str, Any]], output_path: Path):
    """
    Create an HTML visualization with Folium.
    
    Args:
        results: List of detection results
        output_path: Output HTML file path
    """
    try:
        import folium
    except ImportError:
        logger.error("Folium is not installed. Install it with: pip install folium")
        return
    
    if not results:
        logger.warning("No results to visualize")
        return
    
    # Compute map center
    lats = [r['lat'] for r in results]
    lons = [r['lon'] for r in results]
    center_lat = sum(lats) / len(lats)
    center_lon = sum(lons) / len(lons)
    
    # Create map
    m = folium.Map(location=[center_lat, center_lon], zoom_start=18)
    
    # Add markers
    for result in results:
        popup_text = (
            f"Frame: {result['frame_index']}<br>"
            f"Time: {result['timestamp']}<br>"
            f"Confidence: {result['confidence']:.2f}<br>"
            f"Track ID: {result['track_id']}"
        )
        
        folium.CircleMarker(
            location=[result['lat'], result['lon']],
            radius=3,
            popup=popup_text,
            color='red',
            fill=True,
            fillColor='red'
        ).add_to(m)
    
    # Save map
    output_path.parent.mkdir(parents=True, exist_ok=True)
    m.save(str(output_path))
    
    logger.info(f"Saved visualization: {output_path}")


def main():
    """Main entry point."""
    args = parse_args()
    
    logger.info("=" * 60)
    logger.info("Sheepgeo: Sheep Detection and Geo-Localization")
    logger.info("=" * 60)
    
    # Validate inputs
    if not args.video.exists():
        logger.error(f"Video file not found: {args.video}")
        return 1
    
    if not args.srt.exists():
        logger.error(f"SRT file not found: {args.srt}")
        return 1
    
    if args.weights != Path("yolov8n.pt") and not args.weights.exists():
        logger.error(f"Weights file not found: {args.weights}")
        return 1
    
    # Process video
    try:
        results = process_video(args)
    except Exception as e:
        logger.exception(f"Error processing video: {e}")
        return 1
    
    # Save outputs
    if not results:
        logger.warning("No detections found")
        return 0
    
    # Determine output paths
    args.out_dir.mkdir(parents=True, exist_ok=True)
    
    if args.out:
        geojson_path = args.out
    else:
        geojson_path = args.out_dir / DEFAULT_GEOJSON_NAME
    
    csv_path = geojson_path.with_suffix('.csv')
    
    # Save GeoJSON
    save_geojson(results, geojson_path)
    
    # Save CSV
    save_csv(results, csv_path)
    
    # Create visualization if requested
    if args.visualize:
        viz_path = geojson_path.with_suffix('.html')
        create_visualization(results, viz_path)
    
    logger.info("=" * 60)
    logger.info("Processing complete!")
    logger.info(f"Total detections: {len(results)}")
    logger.info(f"Output: {geojson_path}")
    logger.info("=" * 60)
    
    return 0


if __name__ == "__main__":
    exit(main())
