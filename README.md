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
