from ultralytics import YOLO
import cv2
#import os
#print("Tracker file exists:", os.path.exists('modules/bytetrack.yaml'))
# YOLO
'''
@software{yolov8_ultralytics,
  author = {Glenn Jocher and Ayush Chaurasia and Jing Qiu},
  title = {Ultralytics YOLOv8},
  version = {8.0.0},
  year = {2023},
  url = {https://github.com/ultralytics/ultralytics},
  orcid = {0000-0001-5950-6979, 0000-0002-7603-6750, 0000-0003-3783-7069},
  license = {AGPL-3.0}}
'''


class SheepDetectROS:

    def __init__(self, weights, tracker, conf=0.3, iou=0.8, agnostic_nms=True, max_det=100, verbose=False, stream=True):
        # Load YOLO with pretrained weights
        self.model = YOLO(weights)
        self.conf = conf
        self.iou = iou
        self.agnostic_nms = agnostic_nms
        self.max_det = max_det
        self.verbose = verbose
        self.stream = stream
        self.tracker = tracker

    def predict(self, frame, gps, attitude=None, gimbal=None, camera=None):

        # decode gps info
        try:
            lat = gps.latitude
            lon = gps.longitude
            alt = gps.altitude
        except:
            lat = 53
            lon = -1.0
            alt = 72

        # Extract drone orientation (default to reasonable values if not provided)
        flight_yaw = attitude.z if attitude else 0.0  # drone yaw in degrees
        gimbal_yaw_relative = gimbal.x if gimbal else 0.0  # gimbal yaw relative to drone
        gimbal_pitch = gimbal.y if gimbal else 0.0    # gimbal pitch in degrees
        
        # Absolute world yaw = drone yaw + gimbal yaw (relative to drone)
        gimbal_yaw = flight_yaw + gimbal_yaw_relative
        
        # Camera specs - Zenmuse H20 (default to these values if not provided)
        focal_length = camera.get('focal_len', 4.5) if camera else 4.5  # mm - Zenmuse H20
        sensor_width = 6.17  # mm - Zenmuse H20
        sensor_height = 4.55  # mm - Zenmuse H20

        # Frame should be openCV format
        h, w = frame.shape[:2]
        poses = []

        # Track sheep in this frame
        results = self.model.track(frame,
                             conf=self.conf,
                             iou=self.iou,
                             agnostic_nms=self.agnostic_nms,
                             max_det=self.max_det,
                             verbose=self.verbose,
                             stream=self.stream,
                             tracker=self.tracker,
                             persist=True)

        # Extract boxes and tracked IDs
        boxes, ids = self.getBoxes(results)
        for box in boxes:
            x, y = self.centroid(box)
            sheep_lat, sheep_lon = get_gps_from_pixel(x, y, w, h,
                       flight_degree=flight_yaw, gimbal_yaw_degree=gimbal_yaw, gimbal_pitch=gimbal_pitch,
                       gps_lat_decimal=lat, gps_lon_decimal=lon,
                       altitude_meters=alt, focal_length_mm=focal_length, sensor_width_mm=sensor_width, sensor_height_mm=sensor_height)
            # Convert lat,lon into pose format (dictionary)
            poses.append(self.makePose(float(sheep_lat), float(sheep_lon)))

        return [ids, poses, boxes]


    def getBoxes(self, results):
        boxes = []
        tid = []
        for result in results:
            if result.boxes is not None:
                for box in result.boxes:
                    track_id = int(box.id.item()) if box.id is not None else -1
                    box = box[0]
                    boxes.append(box)
                    tid.append(track_id)
        return boxes, tid

    def centroid(self, box):
        x1, y1, x2, y2 = box.xyxy.numpy()[0]
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        return cx,cy

    def makePose(self,cx,cy):
        pose = {
            'position': {
                'x': cx,
                'y': cy,
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0
            }
        }
        return pose


def get_gps_from_pixel(pixel_x, pixel_y, image_width, image_height,
                       flight_degree, gimbal_yaw_degree, gimbal_pitch,
                       gps_lat_decimal, gps_lon_decimal,
                       altitude_meters, focal_length_mm, sensor_width_mm, sensor_height_mm):
    """
    Convert pixel coordinates to GPS using ray tracing through tilted camera.
    
    Projects each pixel as a ray from camera, accounts for gimbal tilt/yaw,
    and intersects with ground plane at drone altitude.

    Args:
    - pixel_x, pixel_y: The pixel coordinates in the image.
    - image_width, image_height: The dimensions of the image in pixels.
    - flight_degree: The flight yaw orientation in degrees (not used if gimbal stabilized).
    - gimbal_yaw_degree: The gimbal yaw orientation in degrees.
    - gimbal_pitch: The gimbal pitch angle in degrees (0° = straight down).
    - gps_lat_decimal, gps_lon_decimal: GPS coordinates of drone.
    - altitude_meters: The altitude of the drone in meters (AGL).
    - focal_length_mm: Camera focal length in millimeters (includes digital zoom).
    - sensor_width_mm, sensor_height_mm: Camera sensor size in millimeters.

    Returns:
    - (latitude, longitude): The GPS coordinates corresponding to the pixel location.
    """
    import math
    
    # Convert gimbal angles to radians
    yaw_rad = math.radians(gimbal_yaw_degree)
    pitch_rad = math.radians(gimbal_pitch)
    
    import sys
    
    # Step 1: Convert pixel to normalized camera coordinates (-1 to +1)
    norm_x = (pixel_x - image_width / 2) / image_width * 2   # -1 to +1
    norm_y = (image_height / 2 - pixel_y) / image_height * 2  # -1 to +1 (inverted)
    
    # Step 2: Convert to sensor-space coordinates
    sensor_x = norm_x * sensor_width_mm / 2
    sensor_y = norm_y * sensor_height_mm / 2
    
    # Step 3: Convert to camera-space ray direction using focal length
    ray_x = sensor_x / focal_length_mm
    ray_y = sensor_y / focal_length_mm
    ray_z = 1.0
    
    # Normalize the ray
    ray_length = math.sqrt(ray_x**2 + ray_y**2 + ray_z**2)
    ray_x /= ray_length
    ray_y /= ray_length
    ray_z /= ray_length
    
    # Step 4: Rotate ray by gimbal pitch (rotation around X-axis)
    # pitch: positive = tilted down more, negative = tilted up
    # For -56.1° pitch: we want ray_z to become more negative (looking down)
    cos_pitch = math.cos(pitch_rad)
    sin_pitch = math.sin(pitch_rad)
    ray_x_rot = ray_x
    ray_y_rot = ray_y * cos_pitch + ray_z * sin_pitch  # Note: changed sign to get correct tilt direction
    ray_z_rot = -ray_y * sin_pitch + ray_z * cos_pitch  # Changed to make negative pitch tilt downward
    
    # Step 5: Rotate ray by gimbal yaw (rotation around Z-axis)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    
    # Convert to world frame (x=East, y=North, z=Up)
    ray_north = ray_z_rot * cos_yaw - ray_x_rot * sin_yaw
    ray_east = ray_z_rot * sin_yaw + ray_x_rot * cos_yaw
    ray_up = ray_y_rot
    
    print(f"[GPS_DEBUG] pixel=({pixel_x:.0f},{pixel_y:.0f}) ray_north={ray_north:.3f} ray_east={ray_east:.3f} ray_up={ray_up:.3f}", file=sys.stderr)
    
    # Step 6: Ray-plane intersection with ground
    if abs(ray_up) > 0.0001:
        t = -altitude_meters / ray_up
        
        if t > 0:
            ground_north = ray_north * t
            ground_east = ray_east * t
            print(f"[GPS_DEBUG] t={t:.1f}m ground_N={ground_north:.1f}m ground_E={ground_east:.1f}m", file=sys.stderr)
        else:
            ground_north = 0
            ground_east = 0
            print(f"[GPS_DEBUG] Ray away from ground! t={t:.1f}", file=sys.stderr)
    else:
        ground_north = 0
        ground_east = 0
        print(f"[GPS_DEBUG] Ray parallel to ground!", file=sys.stderr)
    
    # Step 7: Convert ground offset to lat/lon
    lat_change_deg = ground_north / 111320
    lon_meters_per_degree = 40008000 * math.cos(math.radians(gps_lat_decimal)) / 360
    lon_change_deg = ground_east / lon_meters_per_degree
    
    # Final GPS position
    sheep_lat = gps_lat_decimal + lat_change_deg
    sheep_lon = gps_lon_decimal + lon_change_deg
    
    print(f"[GPS_DEBUG] lat_offset={ground_north:.1f}m lon_offset={ground_east:.1f}m", file=sys.stderr)
    
    return sheep_lat, sheep_lon
