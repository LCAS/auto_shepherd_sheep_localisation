"""
Configuration and default parameters for sheepgeo.
"""
from dataclasses import dataclass
from typing import Dict, Tuple
import numpy as np


@dataclass
class CameraConfig:
    """Camera configuration parameters."""
    name: str
    fov_deg: float
    focal_length_mm: float
    sensor_width_mm: float
    sensor_height_mm: float
    image_width_px: int
    image_height_px: int
    f_number: float = 2.8
    
    def get_intrinsics(self) -> np.ndarray:
        """Compute camera intrinsic matrix K from FOV and image size."""
        # Focal length in pixels
        fx = self.image_width_px / (2 * np.tan(np.deg2rad(self.fov_deg) / 2))
        fy = fx  # assume square pixels
        
        # Principal point (image center)
        cx = self.image_width_px / 2.0
        cy = self.image_height_px / 2.0
        
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        return K


# DJI Mini 2 SE camera parameters
# 12MP sensor, 1/2.3" sensor (approx 6.17 x 4.55 mm)
# 24mm equivalent focal length, f/2.8
# Typical video: 2.7K (2720x1530) or 1080p (1920x1080)
MINI_2_SE_2_7K = CameraConfig(
    name="DJI Mini 2 SE (2.7K)",
    fov_deg=83.0,
    focal_length_mm=4.5,  # actual focal length (not equivalent)
    sensor_width_mm=6.17,
    sensor_height_mm=4.55,
    image_width_px=2720,
    image_height_px=1530,
    f_number=2.8
)

MINI_2_SE_1080P = CameraConfig(
    name="DJI Mini 2 SE (1080p)",
    fov_deg=83.0,
    focal_length_mm=4.5,
    sensor_width_mm=6.17,
    sensor_height_mm=4.55,
    image_width_px=1920,
    image_height_px=1080,
    f_number=2.8
)

# DJI Mavic Air 2 camera parameters
# 48MP sensor, 1/2" sensor (approx 6.4 x 4.8 mm)
# 24mm equivalent focal length, f/2.8
# Typical video: 4K (3840x2160) or 1080p (1920x1080)
MAVIC_AIR_2_4K = CameraConfig(
    name="DJI Mavic Air 2 (4K)",
    fov_deg=84.0,
    focal_length_mm=4.5,
    sensor_width_mm=6.4,
    sensor_height_mm=4.8,
    image_width_px=3840,
    image_height_px=2160,
    f_number=2.8
)

MAVIC_AIR_2_1080P = CameraConfig(
    name="DJI Mavic Air 2 (1080p)",
    fov_deg=84.0,
    focal_length_mm=4.5,
    sensor_width_mm=6.4,
    sensor_height_mm=4.8,
    image_width_px=1920,
    image_height_px=1080,
    f_number=2.8
)

# Drone model to camera config mapping
DRONE_CAMERAS: Dict[str, CameraConfig] = {
    "mini_2_se": MINI_2_SE_2_7K,
    "mini_2_se_1080p": MINI_2_SE_1080P,
    "mavic_air_2": MAVIC_AIR_2_4K,
    "mavic_air_2_1080p": MAVIC_AIR_2_1080P,
}


# YOLO detection defaults
DEFAULT_YOLO_MODEL = "yolov8n.pt"  # Nano model, includes 'sheep' class
DEFAULT_CONFIDENCE = 0.25
DEFAULT_IOU_THRESHOLD = 0.45
SHEEP_CLASS_ID = 18  # COCO class ID for sheep


# Ground projection defaults
DEFAULT_AGL_OFFSET_M = 2.0  # Assume ground is 2m below drone altitude
DEFAULT_GROUND_ELEV_M = 0.0  # Sea level by default


# Output defaults
DEFAULT_OUTPUT_DIR = "outputs"
DEFAULT_GEOJSON_NAME = "sheep_positions.geojson"
DEFAULT_CSV_NAME = "sheep_positions.csv"
