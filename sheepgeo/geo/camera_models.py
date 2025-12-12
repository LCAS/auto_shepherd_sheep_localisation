"""
Camera intrinsics and calibration for DJI drones.

This module provides camera models for:
- DJI Mavic Air 2
- DJI Mini 2 SE

It includes functions to compute intrinsic matrices from FOV and image dimensions,
and helpers for camera calibration.
"""
import numpy as np
import logging
from typing import Optional, Tuple
from pathlib import Path
import json

from sheepgeo.config import CameraConfig, DRONE_CAMERAS


logger = logging.getLogger(__name__)


def compute_intrinsics_from_fov(
    fov_deg: float,
    image_width_px: int,
    image_height_px: int
) -> np.ndarray:
    """
    Compute camera intrinsic matrix K from horizontal FOV and image size.
    
    Uses the pinhole camera model:
    K = [[fx,  0, cx],
         [ 0, fy, cy],
         [ 0,  0,  1]]
    
    where:
    - fx, fy: focal length in pixels
    - cx, cy: principal point (typically image center)
    
    Args:
        fov_deg: Horizontal field of view in degrees
        image_width_px: Image width in pixels
        image_height_px: Image height in pixels
        
    Returns:
        3x3 intrinsic matrix K
    """
    # Compute focal length in pixels from FOV
    fx = image_width_px / (2.0 * np.tan(np.deg2rad(fov_deg) / 2.0))
    fy = fx  # Assume square pixels
    
    # Principal point (image center)
    cx = image_width_px / 2.0
    cy = image_height_px / 2.0
    
    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)
    
    logger.debug(f"Computed K from FOV={fov_deg}°: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
    
    return K


def compute_intrinsics_from_focal_length(
    focal_length_mm: float,
    sensor_width_mm: float,
    sensor_height_mm: float,
    image_width_px: int,
    image_height_px: int
) -> np.ndarray:
    """
    Compute camera intrinsic matrix K from physical sensor and focal length.
    
    Args:
        focal_length_mm: Focal length in millimeters
        sensor_width_mm: Sensor width in millimeters
        sensor_height_mm: Sensor height in millimeters
        image_width_px: Image width in pixels
        image_height_px: Image height in pixels
        
    Returns:
        3x3 intrinsic matrix K
    """
    # Compute focal length in pixels
    fx = focal_length_mm * image_width_px / sensor_width_mm
    fy = focal_length_mm * image_height_px / sensor_height_mm
    
    # Principal point (image center)
    cx = image_width_px / 2.0
    cy = image_height_px / 2.0
    
    K = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ], dtype=np.float64)
    
    logger.debug(f"Computed K from focal length={focal_length_mm}mm: fx={fx:.2f}, fy={fy:.2f}")
    
    return K


def get_camera_config(
    drone_model: str,
    image_width_px: Optional[int] = None,
    image_height_px: Optional[int] = None
) -> CameraConfig:
    """
    Get camera configuration for a drone model.
    
    If image dimensions are provided, the configuration will be adjusted
    to match them (useful for different video resolutions).
    
    Args:
        drone_model: Drone model key (e.g., 'mavic_air_2', 'mini_2_se')
        image_width_px: Override image width
        image_height_px: Override image height
        
    Returns:
        CameraConfig object
        
    Raises:
        ValueError: If drone model is not recognized
    """
    if drone_model not in DRONE_CAMERAS:
        raise ValueError(f"Unknown drone model: {drone_model}. "
                        f"Available models: {list(DRONE_CAMERAS.keys())}")
    
    config = DRONE_CAMERAS[drone_model]
    
    # If image dimensions are provided, create a new config with those dimensions
    if image_width_px is not None and image_height_px is not None:
        from dataclasses import replace
        config = replace(
            config,
            image_width_px=image_width_px,
            image_height_px=image_height_px
        )
        logger.info(f"Adjusted camera config for {drone_model} to {image_width_px}x{image_height_px}")
    
    return config


def load_calibration_file(calib_path: Path) -> dict:
    """
    Load camera calibration from a YAML or JSON file.
    
    Expected format:
    {
        "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
        "distortion_coeffs": [k1, k2, p1, p2, k3],  # optional
        "image_size": [width, height],
        "fov_deg": 84.0  # optional
    }
    
    Args:
        calib_path: Path to calibration file
        
    Returns:
        Dictionary with calibration parameters
    """
    calib_path = Path(calib_path)
    
    if not calib_path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calib_path}")
    
    # Load JSON or YAML
    if calib_path.suffix in ['.json']:
        with open(calib_path, 'r') as f:
            calib = json.load(f)
    elif calib_path.suffix in ['.yaml', '.yml']:
        try:
            import yaml
            with open(calib_path, 'r') as f:
                calib = yaml.safe_load(f)
        except ImportError:
            raise ImportError("PyYAML is required to load .yaml calibration files. "
                            "Install it with: pip install pyyaml")
    else:
        raise ValueError(f"Unsupported calibration file format: {calib_path.suffix}")
    
    logger.info(f"Loaded calibration from {calib_path}")
    
    return calib


def create_custom_camera_config(
    name: str,
    fov_deg: Optional[float] = None,
    focal_length_mm: Optional[float] = None,
    sensor_width_mm: Optional[float] = None,
    sensor_height_mm: Optional[float] = None,
    image_width_px: int = 1920,
    image_height_px: int = 1080,
    camera_matrix: Optional[np.ndarray] = None
) -> CameraConfig:
    """
    Create a custom camera configuration.
    
    You can specify either:
    1. FOV in degrees (and optionally focal length + sensor size)
    2. Camera matrix directly
    
    Args:
        name: Camera name
        fov_deg: Horizontal field of view in degrees
        focal_length_mm: Focal length in millimeters
        sensor_width_mm: Sensor width in millimeters
        sensor_height_mm: Sensor height in millimeters
        image_width_px: Image width in pixels
        image_height_px: Image height in pixels
        camera_matrix: 3x3 camera intrinsic matrix (overrides other params)
        
    Returns:
        CameraConfig object
    """
    if camera_matrix is not None:
        # Extract parameters from camera matrix
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        # Compute FOV from focal length
        fov_deg = 2.0 * np.rad2deg(np.arctan(image_width_px / (2.0 * fx)))
        
        # Use default sensor size if not provided
        if focal_length_mm is None:
            focal_length_mm = 4.5
        if sensor_width_mm is None:
            sensor_width_mm = 6.0
        if sensor_height_mm is None:
            sensor_height_mm = 4.5
    
    elif fov_deg is None:
        raise ValueError("Either fov_deg or camera_matrix must be provided")
    
    # Use defaults for physical sensor if not provided
    if focal_length_mm is None:
        focal_length_mm = 4.5
    if sensor_width_mm is None:
        sensor_width_mm = 6.0
    if sensor_height_mm is None:
        sensor_height_mm = 4.5
    
    config = CameraConfig(
        name=name,
        fov_deg=fov_deg,
        focal_length_mm=focal_length_mm,
        sensor_width_mm=sensor_width_mm,
        sensor_height_mm=sensor_height_mm,
        image_width_px=image_width_px,
        image_height_px=image_height_px
    )
    
    logger.info(f"Created custom camera config: {name}")
    
    return config
