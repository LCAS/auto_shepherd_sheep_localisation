# Map Visualiser Node

## Overview
Real-time web-based visualization system for sheep detection, tracking, and clustering from aerial drone footage. Displays live sheep positions, movement trails, cluster formations, camera field of view, and detection video feed on an interactive map interface.

## Features Added (22 Dec 2025)

### Clustering System
- **Sheep Clustering**: Automatically groups nearby sheep (within ~15m radius) into clusters
- **Cluster Visualization**: Purple circular markers scaled by cluster size
- **Cluster Sidebar**: Displays cluster count, ID, location, and sheep count per cluster
- **Toggle Control**: Show/hide cluster markers independently

### Map Overlays
- **Sheep Trails**: Green polylines showing historical movement paths (120 points max)
- **Camera FOV**: Blue polygon showing drone camera field of view footprint
- **Toggle Controls**: Independent visibility controls for trails, clusters, and FOV

### Trail History Management
- **History Limit**: Capped at 120 position points per sheep (~4 seconds at max frame rate)
- **Memory Efficient**: Automatic pruning of old positions to prevent memory growth

### Map Layers
- **Satellite View**: Default high-resolution imagery (Esri)
- **Street Map**: OpenStreetMap standard view
- **Hybrid**: Satellite with street overlay

## ROS2 Topics

### Subscribed Topics
- `/drone/gps` (sensor_msgs/NavSatFix): Drone GPS position for map centering and FOV calculation
- `/drone/attitude` (geometry_msgs/Vector3Stamped): Drone yaw/pitch/roll for FOV transformation
- `/drone/gimbal` (geometry_msgs/Vector3Stamped): Gimbal orientation for FOV calculation
- `/sheep_paths` (nav_msgs/Path): Individual sheep GPS positions with tracking IDs
- `/sheep_clusters` (nav_msgs/Path): Clustered sheep centroids with cluster size in pose.z
- `/sheep_detections` (sensor_msgs/Image): Annotated video frames with bounding boxes

### Topic Details

#### `/sheep_paths`
Published by: `detect_sheep.py`
- Each PoseStamped contains:
  - `header.frame_id`: Sheep tracking ID (string)
  - `pose.position.x`: Latitude (decimal degrees)
  - `pose.position.y`: Longitude (decimal degrees)

#### `/sheep_clusters`
Published by: `detect_sheep.py`
- Each PoseStamped contains:
  - `header.frame_id`: Cluster ID (e.g., "cluster_1")
  - `pose.position.x`: Cluster centroid latitude
  - `pose.position.y`: Cluster centroid longitude
  - `pose.position.z`: Number of sheep in cluster (float)

#### `/sheep_detections`
Published by: `detect_sheep.py`
- Annotated image frames with:
  - Bounding boxes around detected sheep
  - Sheep tracking IDs
  - GPS coordinates overlaid on detections

## Web Interface

### Access
Open browser to: `http://localhost:8080`

### Sidebar Controls
- **Drone Position**: Real-time GPS coordinates and altitude
- **Map View**: Switch between satellite, street, and hybrid layers
- **Overlays**: Toggle visibility of trails, clusters, and camera FOV
- **Detected Sheep**: List of all tracked sheep with coordinates
- **Sheep Clusters**: List of clusters with location and size
- **Legend**: Color coding for drone (blue), sheep (orange), clusters (purple)

### Video Feed
Bottom panel displays live annotated detection feed with bounding boxes and labels.

## Node Configuration

### Trail History Length
Edit `max_history_points` in `map_visualiser_node.py`:
```python
self.max_history_points = 120  # Number of position points to retain per sheep
```

### Clustering Parameters
Edit clustering radius in `detect_sheep.py`:
```python
def cluster_sheep(self, positions, radius_m=15.0):  # Radius in meters
```

## Architecture

### Backend (Python/ROS2)
- `map_visualiser_node.py`: ROS2 node handling topic subscriptions and data processing
- Flask web server with SocketIO for real-time updates
- Camera FOV calculation using gimbal angles and drone telemetry
- History management with configurable point limits

### Frontend (HTML/JavaScript)
- `web_templates/map.html`: Interactive Leaflet.js map interface
- Real-time WebSocket updates via Socket.IO
- Dynamic marker rendering and layer management
- MJPEG video stream display

## Dependencies
- ROS2 (Humble/Foxy)
- Flask & Flask-SocketIO
- cv_bridge
- Leaflet.js
- Socket.IO client

## Launch
```bash
ros2 run auto_shepherd_sheep_localisation_ros2 map_visualiser_node.py
```

## Sheep Radar Boids analysis

The package includes a separate `boids_analysis_node.py`. It consumes the
timestamped visible-track snapshots on `/sheep_paths`, converts GPS to a fixed
local east/north metric frame, maintains bounded causal motion histories, and
publishes versioned JSON results on `/sheep/boids_analysis`. It fits
non-negative rolling gains for cohesion, alignment, separation, and (when a
field boundary is available) boundary avoidance. Coefficients are model gains,
not percentages; the feature vectors and fitted acceleration target are in
`m/s^2`.

The injected tmule launch starts the visualiser, analysis node, detector, and
replay loader. RViz is intentionally not part of this launch:

```bash
cd /home/carrot/code/auto_shepherd/auto_shepherd_sheep_localisation/docker
docker compose run --rm auto_shepherd_sheep_localisation_ros2_humble bash

cd /home/ros/base_ws/src/auto_shepherd_sheep_localisation_ros2/tmule
tmule -c injected.tmule.yaml launch
```

Open `http://localhost:8080`. The dashboard shows a warming-up state until the
configured 30-second minimum window is available. Results are persisted in
SQLite at `/home/ros/data/boids.sqlite3`, mounted from the repository's
`data/` directory, and can be read from `/boids/history` or exported at
`/boids/export.csv`.

The **Replay video** section lists MP4/SRT pairs from
`detection_process/models/videos`, `models/samples/sample_clip`, and other
sample subfolders. Files are listed only when the MP4 has a matching SRT with
the same filename stem. Selecting a replay restarts the loader, detector
tracking, dashboard trails, and Boids segment from frame one.

The video timeline includes **Pause/Play** and **Reset tracking** controls.
Reset tracking restarts the selected replay at frame one and clears tracker
IDs, sheep trails, Boids history, and map overlay vectors.

Click a detected sheep marker to select it. When sufficient independent history
exists, the analysis panel will show that track's estimate; the optional **Boids vectors** overlay draws
the current cohesion, alignment, separation, and boundary vectors for every
currently visible detected sheep. The selected sheep still controls the
per-track estimate shown in the analysis panel. Blue, green, red, and purple
arrows represent cohesion, alignment, separation, and boundary influence
respectively. Arrow lengths are display-scaled (8 metres per `m/s^2`) and are
not calibrated force magnitudes.

For a detector-free seeded smoke demonstration, run the visualiser and
analysis node as above, then in another container shell run:

```bash
ros2 run auto_shepherd_sheep_localisation_ros2 boids_demo_node.py
```

Use `--ros-args -p minimum_window_s:=5.0 -p minimum_samples:=20` on the
analysis node for a short demo only; those are not recommended real-data
defaults. The synthetic stream is labelled `source_type=simulation` and is
not a welfare or health baseline.

## Technical Details

### FOV Calculation
- Uses pinhole camera model with sensor dimensions
- Applies gimbal pitch/yaw rotations
- Ray-ground plane intersection for corner GPS coordinates
- Camera specs: Zenmuse H20 (4.5mm focal length, 1920x1080)

### Clustering Algorithm
- Simple agglomerative clustering by Euclidean distance
- Radius-based threshold (~15m default)
- Weighted centroid update as sheep join clusters
- Real-time updates on every detection frame

### Data Flow
1. `detect_sheep.py` processes drone images with YOLO
2. Converts pixel detections to GPS coordinates
3. Publishes individual positions and clusters
4. `map_visualiser_node.py` aggregates data
5. WebSocket pushes updates to browser
6. JavaScript renders markers and overlays on map
