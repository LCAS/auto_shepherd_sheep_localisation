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

    def __init__(self, weights, conf=0.3, iou=0.8, agnostic_nms=True, max_det=100, verbose=False, stream=True):
        # Load YOLO with pretrained weights
        self.model = YOLO(weights) 
        self.conf = conf  
        self.iou = iou
        self.agnostic_nms = agnostic_nms
        self.max_det = max_det
        self.verbose = verbose
        self.stream = stream

    def predict(self, frame, gps):
        
        # decode gps info
        try:
            lat = gps.latitude
            lon = gps.longitude
            alt = gps.altitude
        except:
            lat = 53
            lon = -1.0
            alt = 72

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
                             tracker='modules/bytetrack.yaml',
                             persist=True)

        # Extract boxes and tracked IDs
        boxes, ids = self.getBoxes(results)
        for box in boxes:
            x,y = self.centroid(box)
            lat, lon =  get_gps_from_pixel(x, y, w, h, 
                       flight_degree=30, gimbal_degree=10, gps_lat_decimal=lat, gps_lon_decimal = lon, 
                       altitude_meters=alt, focal_length_mm=58, sensor_width_mm=10, sensor_height_mm=10)
            # Convert lat,lon into pose format (dictionary)
            poses.append(self.makePose(lat.item(),lon.item()))

        return [ids, poses]
    
    
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
                       flight_degree, gimbal_degree, gps_lat_decimal, gps_lon_decimal, 
                       altitude_meters, focal_length_mm, sensor_width_mm, sensor_height_mm):
    """
    Convert pixel coordinates back to GPS latitude and longitude.
    
    Args:
    - pixel_x, pixel_y: The pixel coordinates in the image.
    - image_width, image_height: The dimensions of the image in pixels.
    - flight_degree: The flight yaw orientation in degrees.
    - gimbal_degree: The gimbal yaw orientation in degrees.
    - gps_lat_decimal, gps_lon_decimal: GPS coordinates (latitude and longitude) of the image center.
    - altitude_meters: The altitude of the drone in meters.
    - focal_length_mm: Camera focal length in millimeters.
    - sensor_width_mm, sensor_height_mm: Camera sensor size in millimeters.
    
    Returns:
    - (latitude, longitude): The GPS coordinates corresponding to the pixel location.
    """
    import math
    # **Calculate Horizontal and Vertical FOV using Focal Length**
    fov_rad_h = 2 * math.atan((sensor_width_mm / (2 * focal_length_mm)))  
    fov_rad_v = 2 * math.atan((sensor_height_mm / (2 * focal_length_mm)))

    # **Calculate Ground Coverage**
    ground_width_meters = 2 * altitude_meters * math.tan(fov_rad_h / 2)
    ground_height_meters = 2 * altitude_meters * math.tan(fov_rad_v / 2)

    # **Calculate Ground Sample Distance (GSD)**
    gsd_meters_per_pixel_x = ground_width_meters / image_width
    gsd_meters_per_pixel_y = ground_height_meters / image_height

    # Offset the pixel coordinates relative to the center of the image
    corrected_pixel_x = pixel_x - (image_width / 2)
    corrected_pixel_y = (image_height / 2) - pixel_y  # Invert Y-axis for image coordinates

    # Convert pixel offsets to real-world distances in meters
    corrected_lon_change = corrected_pixel_x * gsd_meters_per_pixel_x
    corrected_lat_change = corrected_pixel_y * gsd_meters_per_pixel_y

    # Convert gimbal orientation to radians
    gimbal_radians = math.radians(float(gimbal_degree))

    # Reverse the displacement adjustments for gimbal orientation
    lon_change = corrected_lon_change * math.cos(gimbal_radians) + corrected_lat_change * math.sin(gimbal_radians)
    lat_change = -corrected_lon_change * math.sin(gimbal_radians) + corrected_lat_change * math.cos(gimbal_radians)

    
    # Convert the real-world displacements back to degrees
    latitude = gps_lat_decimal + (lat_change / 111320)  # Convert meters to degrees for latitude
    longitude = gps_lon_decimal + (lon_change / (40008000 * math.cos(math.radians(latitude)) / 360))  # Convert meters to degrees for longitude

    return latitude, longitude