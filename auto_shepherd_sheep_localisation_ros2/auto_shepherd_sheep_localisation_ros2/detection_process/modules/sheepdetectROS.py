from ultralytics import YOLO
import cv2
import numpy as np

# import os
# print("Tracker file exists:", os.path.exists('modules/bytetrack.yaml'))
# YOLO
"""
@software{yolov8_ultralytics,
  author = {Glenn Jocher and Ayush Chaurasia and Jing Qiu},
  title = {Ultralytics YOLOv8},
  version = {8.0.0},
  year = {2023},
  url = {https://github.com/ultralytics/ultralytics},
  orcid = {0000-0001-5950-6979, 0000-0002-7603-6750, 0000-0003-3783-7069},
  license = {AGPL-3.0}}
"""


class SheepDetectROS:

    def __init__(
        self,
        weights,
        tracker,
        conf=0.3,
        iou=0.8,
        agnostic_nms=True,
        max_det=100,
        verbose=False,
        stream=True,
        imgsz=640,
        tiling_mode="full",
    ):
        # Load YOLO with pretrained weights
        self.model = YOLO(weights)
        self.conf = conf
        self.iou = iou
        self.agnostic_nms = agnostic_nms
        self.max_det = max_det
        self.verbose = verbose
        self.stream = stream
        self.tracker = tracker
        self.imgsz = 640
        self.set_inference_size(imgsz)
        self.tiling_mode = "full"
        self.set_tiling_mode(tiling_mode)
        self.last_inference_warning = None

    def set_inference_size(self, imgsz):
        """Set the Ultralytics inference canvas size for future frames."""
        try:
            value = int(imgsz)
        except (TypeError, ValueError):
            raise ValueError("YOLO inference size must be an integer")
        if value not in {640, 960, 1280}:
            raise ValueError("YOLO inference size must be one of 640, 960, or 1280")
        self.imgsz = value

    def set_tiling_mode(self, mode):
        """Select full-frame inference or the experimental fixed 2x2 mode."""
        value = str(mode or "full").strip().lower()
        if value not in {"full", "2x2"}:
            raise ValueError("Inference mode must be 'full' or '2x2'")
        self.tiling_mode = value

    @staticmethod
    def _mapped_box(coordinates):
        """Provide the small box interface used by detect_sheep.py."""
        class ArrayTensor:
            def __init__(self, values):
                self._values = np.asarray([values], dtype=np.float32)

            def numpy(self):
                return self._values

        class MappedBox:
            def __init__(self, values):
                self.xyxy = ArrayTensor(values)

        return MappedBox(coordinates)

    @staticmethod
    def _make_2x2_tiles(frame, tile_size=640):
        """Build a 2x2 640px tile canvas and return inverse transforms.

        Each source quadrant is letterboxed into a 640x640 tile. The resulting
        1280x1280 canvas is deliberately passed to YOLO at imgsz=1280 so that
        small objects receive more pixels than full-frame 640 inference.
        """
        height, width = frame.shape[:2]
        canvas = np.zeros((tile_size * 2, tile_size * 2, 3), dtype=frame.dtype)
        transforms = []
        for row in range(2):
            for column in range(2):
                x0 = (width * column) // 2
                x1 = (width * (column + 1)) // 2
                y0 = (height * row) // 2
                y1 = (height * (row + 1)) // 2
                crop = frame[y0:y1, x0:x1]
                crop_height, crop_width = crop.shape[:2]
                scale = min(tile_size / crop_width, tile_size / crop_height)
                resized_width = max(1, round(crop_width * scale))
                resized_height = max(1, round(crop_height * scale))
                resized = cv2.resize(crop, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
                pad_x = (tile_size - resized_width) // 2
                pad_y = (tile_size - resized_height) // 2
                canvas[
                    row * tile_size + pad_y:row * tile_size + pad_y + resized_height,
                    column * tile_size + pad_x:column * tile_size + pad_x + resized_width,
                ] = resized
                transforms.append((x0, y0, scale, pad_x, pad_y))
        return canvas, transforms

    def _track(self, frame, imgsz):
        return self.model.track(
            frame,
            conf=self.conf,
            iou=self.iou,
            agnostic_nms=self.agnostic_nms,
            max_det=self.max_det,
            verbose=self.verbose,
            stream=self.stream,
            tracker=self.tracker,
            persist=True,
            imgsz=imgsz,
        )

    def _get_tiled_boxes(self, results, transforms, tile_size=640):
        boxes = []
        ids = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                track_id = int(box.id.item()) if box.id is not None else -1
                coordinates = box.xyxy.detach().cpu().numpy()[0]
                centre_x = (coordinates[0] + coordinates[2]) / 2.0
                centre_y = (coordinates[1] + coordinates[3]) / 2.0
                column = min(1, max(0, int(centre_x // tile_size)))
                row = min(1, max(0, int(centre_y // tile_size)))
                x0, y0, scale, pad_x, pad_y = transforms[row * 2 + column]
                mapped = [
                    (coordinates[0] - column * tile_size - pad_x) / scale + x0,
                    (coordinates[1] - row * tile_size - pad_y) / scale + y0,
                    (coordinates[2] - column * tile_size - pad_x) / scale + x0,
                    (coordinates[3] - row * tile_size - pad_y) / scale + y0,
                ]
                boxes.append(self._mapped_box(mapped))
                ids.append(track_id)
        return boxes, ids

    def reset_tracker(self):
        """Reset Ultralytics/ByteTrack state without reloading model weights."""
        predictor = getattr(self.model, "predictor", None)
        if predictor is None:
            return
        for tracker in getattr(predictor, "trackers", []) or []:
            reset = getattr(tracker, "reset", None)
            if callable(reset):
                reset()
        # Keep the tracker list itself: Ultralytics' callback expects the
        # existing list to contain one tracker, while reset() clears its
        # active tracks and ID counter.
        if hasattr(predictor, "vid_path"):
            predictor.vid_path = [None]

    def set_model(self, weights):
        """Load a new detector model for subsequent frames."""
        self.model = YOLO(weights)

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
        gimbal_yaw_relative = (
            gimbal.x if gimbal else 0.0
        )  # gimbal yaw relative to drone
        gimbal_pitch = gimbal.y if gimbal else 0.0  # gimbal pitch in degrees

        # Absolute world yaw = drone yaw + gimbal yaw (relative to drone)
        gimbal_yaw = flight_yaw + gimbal_yaw_relative

        # Camera specs - Zenmuse H20 (default to these values if not provided)
        focal_length = (
            camera.get("focal_len", 4.5) if camera else 4.5
        )  # mm - Zenmuse H20
        sensor_width = 5.4 # 6.17  # mm - Zenmuse H20
        sensor_height = 3.0 # 3.47 # 4.55  # mm - Zenmuse H20

        # Frame should be openCV format
        h, w = frame.shape[:2]
        poses = []

        self.last_inference_warning = None
        if self.tiling_mode == "2x2":
            tiled_frame, transforms = self._make_2x2_tiles(frame)
            try:
                # Tiled mode uses a fixed 1280 canvas so the four 640px tiles
                # are not immediately downscaled back to the full-frame size.
                results = self._track(tiled_frame, imgsz=1280)
                boxes, ids = self._get_tiled_boxes(results, transforms)
            except Exception as exc:
                # Keep the live dashboard alive if the GTX 1060 cannot fit
                # the experimental tiled model in memory.
                self.reset_tracker()
                self.tiling_mode = "full"
                self.last_inference_warning = f"2x2 tiling failed; fell back to full-frame inference: {exc}"
                results = self._track(frame, imgsz=self.imgsz)
                boxes, ids = self.getBoxes(results)
        else:
            results = self._track(frame, imgsz=self.imgsz)
            boxes, ids = self.getBoxes(results)
        for box in boxes:
            x, y = self.centroid(box)
            sheep_lat, sheep_lon = get_gps_from_pixel(
                x,
                y,
                w,
                h,
                flight_degree=flight_yaw,
                gimbal_yaw_degree=gimbal_yaw,
                gimbal_pitch=gimbal_pitch,
                gps_lat_decimal=lat,
                gps_lon_decimal=lon,
                altitude_meters=alt,
                focal_length_mm=focal_length,
                sensor_width_mm=sensor_width,
                sensor_height_mm=sensor_height,
            )
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
        return cx, cy

    def makePose(self, cx, cy):
        pose = {
            "position": {"x": cx, "y": cy, "z": 0.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        }
        return pose


def get_gps_from_pixel(
    pixel_x,
    pixel_y,
    image_width,
    image_height,
    flight_degree,
    gimbal_yaw_degree,
    gimbal_pitch,
    gps_lat_decimal,
    gps_lon_decimal,
    altitude_meters,
    focal_length_mm,
    sensor_width_mm,
    sensor_height_mm,
):
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
    norm_x = (pixel_x - image_width / 2) / image_width * 2  # -1 to +1
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
    ray_y_rot = (
        ray_y * cos_pitch + ray_z * sin_pitch
    )  # Note: changed sign to get correct tilt direction
    ray_z_rot = (
        -ray_y * sin_pitch + ray_z * cos_pitch
    )  # Changed to make negative pitch tilt downward

    # Step 5: Rotate ray by gimbal yaw (rotation around Z-axis)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)

    # Convert to world frame (x=East, y=North, z=Up)
    ray_north = ray_z_rot * cos_yaw - ray_x_rot * sin_yaw
    ray_east = ray_z_rot * sin_yaw + ray_x_rot * cos_yaw
    ray_up = ray_y_rot

    print(
        f"[GPS_DEBUG] pixel=({pixel_x:.0f},{pixel_y:.0f}) ray_north={ray_north:.3f} ray_east={ray_east:.3f} ray_up={ray_up:.3f}",
        file=sys.stderr,
    )

    # Step 6: Ray-plane intersection with ground
    if abs(ray_up) > 0.0001:
        t = -altitude_meters / ray_up

        if t > 0:
            ground_north = ray_north * t
            ground_east = ray_east * t
            print(
                f"[GPS_DEBUG] t={t:.1f}m ground_N={ground_north:.1f}m ground_E={ground_east:.1f}m",
                file=sys.stderr,
            )
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

    print(
        f"[GPS_DEBUG] lat_offset={ground_north:.1f}m lon_offset={ground_east:.1f}m",
        file=sys.stderr,
    )

    return sheep_lat, sheep_lon
