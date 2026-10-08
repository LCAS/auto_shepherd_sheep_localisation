# Group 1 – ShLAMb (Sheep Localisation and Mapping) 

## Goal:
To develop and validate a robust pipeline for the automated detection and tracking of individual sheep in aerial video data captured by drones, enabling enhanced livestock monitoring and management in open-field environments. 

## Preparation in Advance: 
- Follow the Conda preparation instructions detailed at: [Getting-Started](https://github.com/LCAS/auto_shepherd_sheep_localisation/wiki/Getting-Started)
- Download the UoL Shepherding Dataset, and others from [Aerial-Datasets](https://github.com/LCAS/auto_shepherd_sheep_localisation/wiki/Aerial-Datasets)
- Some materials on Geo-Referencing: [DJI Aerial Geo-Referencing](https://github.com/roboflow/dji-aerial-georeferencing)

## Activities:
### Primary Activity: 
`Sheep Detection` – Develop and optimise a vision-based pipeline for real-time sheep detection and individual tracking from UAV-captured video streams.

### Secondary Activity:
`Geo-Referencing` – Estimate the geo-referenced position of each tracked sheep in a global coordinate frame using drone telemetry and camera pose data, constructing a map of their positions, with respect to the field boundaries. 

### Stretch Activity:
`Synthetic Data` – Work with Group 5 to connect a video stream from the simulation to incorporate synthetic image into the training pipeline. 

## Outcomes:
This work package is expected to deliver a validated end-to-end system capable of detecting and persistently tracking sheep in aerial footage, with output locations accurately mapped to global coordinates. The dataset and methodology will be documented and shared with the research community, laying the groundwork for publication in leading robotics or precision agriculture venues.

## Future Engagement:
Members of the group will be encouraged to engage with future work exploring multi-species detection, behavioural pattern recognition, and integration with animal health monitoring systems.

---

## Recent Updates (22 Dec 2025)

### Clustering & Visualization Enhancements
Added real-time sheep clustering and enhanced map visualization capabilities:

#### New Features
- **Sheep Clustering**: Automatic grouping of nearby sheep (15m radius) with cluster size tracking
- **Interactive Map Overlays**: Toggle controls for sheep trails, clusters, and camera FOV
- **Trail History Management**: Configurable history limit (120 points default) to prevent memory growth
- **Enhanced Sidebar**: Displays cluster count, locations, and sheep per cluster
- **Multi-layer Maps**: Satellite (default), street, and hybrid view options

#### ROS2 Topics
**Published:**
- `/sheep_paths` (nav_msgs/Path): Individual sheep GPS positions with tracking IDs
- `/sheep_clusters` (nav_msgs/Path): Clustered sheep centroids with cluster sizes
- `/sheep_detections` (sensor_msgs/Image): Annotated video with bounding boxes and IDs

**Subscribed:**
- `/drone/gps` (sensor_msgs/NavSatFix): Drone position
- `/drone/attitude` (geometry_msgs/Vector3Stamped): Drone orientation
- `/drone/gimbal` (geometry_msgs/Vector3Stamped): Gimbal angles
- `/drone/image` (sensor_msgs/Image): Raw camera feed

#### Web Interface
Access the live map at `http://localhost:8080` to view:
- Real-time sheep positions (orange markers)
- Movement trails (green polylines)
- Cluster formations (purple markers scaled by size)
- Camera field of view (blue polygon)
- Live annotated video feed
- Drone telemetry and sheep/cluster lists

See [MAP_VISUALIZER_README.md](auto_shepherd_sheep_localisation_ros2/MAP_VISUALIZER_README.md) for detailed documentation.

### Boids analysis launch

The injected tmule configuration now starts the map visualiser, detector,
sample replay, and the rolling Boids analysis node without RViz. From the
Docker directory, start the GPU-enabled service with:

```bash
cd /home/carrot/code/auto_shepherd/auto_shepherd_sheep_localisation/docker
docker compose up -d --build --force-recreate auto_shepherd_sheep_localisation_ros2_humble
docker compose exec --user ros auto_shepherd_sheep_localisation_ros2_humble bash

cd /home/ros/base_ws/src/auto_shepherd_sheep_localisation_ros2/tmule
tmule -c injected.tmule.yaml launch
```

Open `http://localhost:8080`. The farmer-facing dashboard reports
warming-up/quality states, keeps bounded history, persists results in
`data/boids.sqlite3`, and exposes `/boids/export.csv`. Open
`http://localhost:8080/analysis` for filtered research charts and technical
quality information. A detector-free synthetic route is available through
`ros2 run auto_shepherd_sheep_localisation_ros2 boids_demo_node.py`.

Both dashboard pages list the detector models in
`detection_process/models/samples/sample_model`. The model marked **tmule
default** is the one exported by `tmule/injected.tmule.yaml`; choose another
model and press **Start selected video** to reload detection for that replay.
Both pages also provide a **YOLO inference size** selector. It controls the
Ultralytics inference canvas for subsequent frames: 640 is the default and
fastest setting, 960 is the recommended comparison, and 1280 may be slower or
exceed the GTX 1060's available memory. The detector still receives each full
video frame; Ultralytics resizes it internally to the selected canvas. Reset
tracking after changing size if the track IDs become unstable.
An additional **2×2 tiled (experimental)** mode splits each frame into four
640-pixel tiles and maps the detections back to the original frame. It uses a
1280 canvas, may use substantially more GPU memory, and can change IDs when a
sheep crosses a tile boundary. If it cannot run, the detector falls back to
full-frame inference and reports the warning in its ROS log. The dashboard also
shows live GPU utilisation, memory, and temperature through NVIDIA NVML when
`nvidia-ml-py` and the NVIDIA container runtime are available inside the
container.

Publication figures can be generated from an exported CSV:

```bash
python3 -m venv /tmp/sheep-radar-paper
source /tmp/sheep-radar-paper/bin/activate
pip install -r auto_shepherd_sheep_localisation_ros2/requirements.txt
python scripts/generate_boids_figures.py \
  --input boids_analysis.csv \
  --output-dir paper_figures
```

The current MVP estimates cohesion, alignment, and separation only. Field
boundaries remain map context and are not fitted as a Boids feature.

The dashboard labels detections as temporary **Observation** references rather
than verified sheep identities. Internal tracker IDs are retained for data
association, but a tracking failure, reacquisition, or replay reset can change
the displayed reference. In the annotated video, orange boxes and labels mark
current movement-screening candidates; green boxes mark other detections.

### Removing bad replay data

Every time a recording is started, the replay loader creates a new run/session
ID prefixed with the video filename. This keeps repeated runs of the same MP4
separate. When looping is enabled, the video does not stop: tracker state and
Boids state reset at the loop boundary, and the next loop gets another fresh
session ID rather than being mixed into the previous pass. On `/analysis`,
choose a session in **Filter results** and use **Delete selected session data**.
The page requires both a browser confirmation and typing `DELETE`. This removes
the stored Boids results for that session and permanently tombstones the session
so late ROS messages from the bad run cannot recreate it. The source MP4 and
SRT files are never deleted by this action.

Pressing **Reset tracking** also starts a fresh replay session and clears the
live map/Boids graphs. The previous run remains in analysis history until you
delete it explicitly.
