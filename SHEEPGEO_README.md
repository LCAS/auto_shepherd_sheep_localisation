# Sheepgeo Package Documentation

A Python package and CLI for detecting sheep in drone videos and converting pixel coordinates to GPS coordinates using drone telemetry.

## Features

- 🎯 **YOLO-based sheep detection** using Ultralytics YOLOv8/YOLO11
- 📡 **DJI SRT telemetry parsing** for GPS, altitude, and attitude data
- 🌍 **Pixel-to-GPS projection** using camera models and drone pose
- 📊 **GeoJSON and CSV export** for downstream analysis
- 🗺️ **Optional visualization** with Folium
- 🎥 **Supports DJI Mavic Air 2 and Mini 2 SE** drones

## Installation

### Requirements

- Python ≥ 3.10
- OpenCV, NumPy, PyProj, Ultralytics, pydantic, pysrt, rich

### Install from source

```bash
cd /path/to/auto_shepherd_sheep_localisation
pip install -e .
```

### Install with development dependencies

```bash
pip install -e ".[dev]"
```

## Usage

### Basic usage

```bash
sheepgeo \
  --video path/to/DJI_0001.MP4 \
  --srt path/to/DJI_0001.SRT \
  --drone mavic_air_2 \
  --out outputs/sheep_positions.geojson
```

### Full options

```bash
sheepgeo \
  --video path/to/video.mp4 \
  --srt path/to/video.srt \
  --drone mavic_air_2 \
  --weights path/to/custom_yolo.pt \
  --conf 0.25 \
  --iou 0.45 \
  --track \
  --filter-class sheep \
  --agl-offset 2.0 \
  --ground-elev 0.0 \
  --out outputs/sheep_positions.geojson \
  --visualize
```

### CLI Options

#### Input Files
- `--video`: Path to drone video file (MP4, etc.)
- `--srt`: Path to DJI SRT subtitle file with telemetry
- `--drone`: Drone model (`mavic_air_2`, `mavic_air_2_1080p`, `mini_2_se`, `mini_2_se_1080p`)

#### YOLO Detection
- `--weights`: Path to YOLO weights (default: `yolov8n.pt`)
- `--conf`: Confidence threshold (default: 0.25)
- `--iou`: IoU threshold for NMS (default: 0.45)
- `--track`: Enable ByteTrack object tracking
- `--filter-class`: Filter to specific classes (default: `sheep`)

#### Projection Parameters
- `--agl-offset`: Height above ground in meters (default: 2.0)
- `--ground-elev`: Ground elevation in meters (default: 0.0)
- `--sensor-fov-deg`: Override camera FOV
- `--focal-px`: Override focal length in pixels
- `--calib`: Path to camera calibration file (YAML/JSON)

#### Output
- `--out`: Output GeoJSON path
- `--out-dir`: Output directory (default: `outputs/`)
- `--visualize`: Generate HTML map visualization

#### Processing
- `--max-frames`: Limit number of frames to process
- `--debug`: Enable debug logging

## Output Format

### GeoJSON

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [-1.1234, 52.5678, 45.2]
      },
      "properties": {
        "frame_index": 42,
        "timestamp": "0:00:01.400000",
        "confidence": 0.87,
        "bbox_xyxy": [100, 200, 150, 250],
        "image_xy": [125, 225],
        "track_id": 3,
        "source_video": "DJI_0001.MP4",
        "drone_model": "mavic_air_2"
      }
    }
  ]
}
```

### CSV

```csv
frame_index,timestamp,lat,lon,alt_est,confidence,image_x,image_y,bbox_x1,bbox_y1,bbox_x2,bbox_y2,track_id,source_video,drone_model
42,0:00:01.400000,52.5678,-1.1234,45.2,0.87,125,225,100,200,150,250,3,DJI_0001.MP4,mavic_air_2
```

## Architecture

```
sheepgeo/
  __init__.py
  config.py              # Camera configs, defaults
  cli.py                 # Command-line interface
  io/
    srt_reader.py        # DJI SRT parser
  yolo/
    detect.py            # YOLO detection wrapper
  geo/
    camera_models.py     # Camera intrinsics
    projection.py        # Pixel → GPS projection
```

## Camera Models

The package includes pre-configured camera models for:

### DJI Mavic Air 2
- 48MP sensor (1/2" CMOS)
- 24mm equivalent focal length
- FOV: ~84°
- Supported resolutions: 4K (3840×2160), 1080p (1920×1080)

### DJI Mini 2 SE
- 12MP sensor (1/2.3" CMOS)
- 24mm equivalent focal length
- FOV: ~83°
- Supported resolutions: 2.7K (2720×1530), 1080p (1920×1080)

## Projection Method

The package uses a standard pinhole camera model with the following pipeline:

1. **Pixel → Camera Ray**: Convert pixel (u, v) to normalized 3D ray using camera intrinsics K
2. **Camera → World**: Rotate ray using drone attitude (yaw, pitch, roll) and gimbal angles
3. **Ray-Plane Intersection**: Intersect world ray with ground plane (flat or DEM)
4. **ENU → WGS84**: Convert local ENU coordinates to GPS (lat, lon)

### Assumptions

- **Ground plane**: Assumes flat ground at `drone_altitude - agl_offset` by default
- **Gimbal fallbacks**: If gimbal data is missing, assumes nadir (pitch = -90°)
- **Coordinate system**: Uses ENU (East-North-Up) with pyproj for geodetic conversion

## Testing

Run the test suite:

```bash
pytest tests/ -v
```

Run specific test modules:

```bash
pytest tests/test_srt_reader.py -v
pytest tests/test_projection.py -v
pytest tests/test_detect_stub.py -v
```

## Examples

### Example 1: Basic detection with default model

```bash
sheepgeo \
  --video drone_footage.mp4 \
  --srt drone_footage.srt \
  --drone mini_2_se \
  --out results/sheep.geojson
```

### Example 2: Custom model with tracking

```bash
sheepgeo \
  --video flight_002.mp4 \
  --srt flight_002.srt \
  --drone mavic_air_2 \
  --weights models/sheep_finetuned_yolov8m.pt \
  --conf 0.3 \
  --track \
  --visualize
```

## Python API

You can also use sheepgeo as a Python library:

```python
from pathlib import Path
from sheepgeo.io.srt_reader import SrtReader
from sheepgeo.yolo.detect import SheepDetector
from sheepgeo.geo.camera_models import get_camera_config
from sheepgeo.geo.projection import project_detection_bbox_center

# Load telemetry
srt_reader = SrtReader(Path("video.srt"))
telemetry_list = srt_reader.parse_all()

# Initialize detector
detector = SheepDetector(weights_path="yolov8n.pt")

# Get camera config
camera_config = get_camera_config("mavic_air_2", 1920, 1080)

# Process frame
import cv2
frame = cv2.imread("frame.jpg")
detections = detector.detect_frame(frame)

# Project to GPS
for det in detections:
    result = project_detection_bbox_center(
        det['bbox_xyxy'],
        camera_config,
        telemetry_list[0]
    )
    if result:
        u, v, lat, lon, alt = result
        print(f"Sheep at ({lat:.6f}, {lon:.6f})")
```

## References

- Pinhole camera model: https://en.wikipedia.org/wiki/Pinhole_camera_model
- DJI aerial georeferencing: https://github.com/roboflow/dji-aerial-georeferencing
- Ultralytics YOLOv8: https://github.com/ultralytics/ultralytics
