# 🚁 📸 Drone Data Node

Publishes DJI video-aligned telemetry plus video frames from SRT+MP4.

## What it does
- Finds sample MP4/SRT or uses provided paths
- Parses DJI SRT for camera/gps/drone/gimbal fields
- Streams video frames and publishes synchronized telemetry at video FPS
- Optional looping playback

## Run (sample files)
```bash
cd ~/base_ws
colcon build --packages-select auto_shepherd_sheep_localisation_ros2
source install/setup.bash
ros2 run auto_shepherd_sheep_localisation_ros2 drone_data_node
```

## Run (custom files / options)
```bash
ros2 run auto_shepherd_sheep_localisation_ros2 drone_data_node --ros-args \
  -p video_path:=/path/to/video.mp4 \
  -p srt_path:=/path/to/video.srt \
  -p topic_name:=drone \
  -p loop_video:=true
```

## Published topics
- `/drone` (`std_msgs/String`: JSON with all fields)
- `/drone/gps` (`sensor_msgs/NavSatFix`: uses rel_alt, relative altitude)
- `/drone/relative_altitude` (`std_msgs/Float32`: relative altitude in meters)
- `/drone/absolute_altitude` (`std_msgs/Float32`: absolute altitude in meters)
- `/drone/speed` (`geometry_msgs/Vector3Stamped`: drone_speedx/y/z)
- `/drone/attitude` (`geometry_msgs/Vector3Stamped`: drone_yaw/pitch/roll)
- `/drone/gimbal` (`geometry_msgs/Vector3Stamped`: gb_yaw/pitch/roll, camera gimbal data)
- `/drone/camera` (`std_msgs/String`: iso/shutter/fnum/ev/focal_len/dzoom)
- `/drone/image` (`sensor_msgs/Image`: raw frames)

## Notes
- rel_alt and abs_alt are included in the JSON on `/drone`; only abs_alt is set in NavSatFix.
- The node syncs telemetry to video time; enable loop_video to repeat playback.
