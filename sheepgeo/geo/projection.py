"""
Pixel-to-GPS projection using camera model and drone telemetry.

This module implements the complete pipeline for converting pixel coordinates
in drone video frames to GPS coordinates (latitude, longitude) on the ground.

Pipeline:
1. Pixel (u, v) → Camera ray (normalized 3D vector)
2. Camera ray → World ray (using drone + gimbal rotation)
3. World ray → Ground intersection (ray-plane or ray-DEM)
4. ENU coordinates → WGS84 GPS (lat, lon)
"""
import numpy as np
import logging
from typing import Tuple, Optional
from pyproj import Transformer

from sheepgeo.config import CameraConfig, DEFAULT_AGL_OFFSET_M, DEFAULT_GROUND_ELEV_M
from sheepgeo.io.srt_reader import SrtTelemetry


logger = logging.getLogger(__name__)


def pixel_to_camera_ray(
    pixel_u: float,
    pixel_v: float,
    K: np.ndarray
) -> np.ndarray:
    """
    Convert pixel coordinates to a normalized camera ray.
    
    Uses the pinhole camera model:
        x_camera = K^{-1} * [u, v, 1]^T
    
    The resulting ray is normalized to unit length.
    
    Args:
        pixel_u: Pixel x-coordinate (horizontal)
        pixel_v: Pixel y-coordinate (vertical)
        K: 3x3 camera intrinsic matrix
        
    Returns:
        3D normalized ray vector in camera frame [x, y, z]
    """
    # Homogeneous pixel coordinates
    pixel_homog = np.array([pixel_u, pixel_v, 1.0])
    
    # Apply inverse camera matrix
    K_inv = np.linalg.inv(K)
    ray_camera = K_inv @ pixel_homog
    
    # Normalize to unit vector
    ray_camera = ray_camera / np.linalg.norm(ray_camera)
    
    return ray_camera


def build_rotation_matrix(
    yaw_deg: float,
    pitch_deg: float,
    roll_deg: float
) -> np.ndarray:
    """
    Build a 3D rotation matrix from Euler angles (yaw, pitch, roll).
    
    Convention for drone/camera frame:
    - Yaw: rotation around Z axis (vertical), 0° = North, 90° = East
    - Pitch: rotation around Y axis (lateral), positive = nose/camera up
    - Roll: rotation around X axis (longitudinal), positive = right side down
    
    For camera with gimbal pitch -90° (nadir), we want camera forward (Z-axis)
    to point downward in world frame (negative Z).
    
    Args:
        yaw_deg: Yaw angle in degrees (0° = North, 90° = East)
        pitch_deg: Pitch angle in degrees (positive = up)
        roll_deg: Roll angle in degrees (positive = right down)
        
    Returns:
        3x3 rotation matrix
    """
    # Convert to radians
    yaw = np.deg2rad(yaw_deg)
    pitch = np.deg2rad(pitch_deg)
    roll = np.deg2rad(roll_deg)
    
    # Rotation around Z axis (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    
    # Rotation around Y axis (pitch)
    # For nadir camera: pitch should rotate camera from horizontal to downward
    Ry = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [ 0,             1, 0            ],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation around X axis (roll)
    Rx = np.array([
        [1, 0,            0           ],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    
    # Combined rotation: R = Rz * Ry * Rx (ZYX Euler angles)
    R = Rz @ Ry @ Rx
    
    return R


def compose_camera_rotation(
    drone_yaw_deg: float,
    drone_pitch_deg: float,
    drone_roll_deg: float,
    gimbal_yaw_deg: float,
    gimbal_pitch_deg: float,
    gimbal_roll_deg: float
) -> np.ndarray:
    """
    Compose the full camera-to-world rotation from drone and gimbal attitudes.
    
    The camera is mounted on a 3-axis gimbal on the drone. The gimbal pitch
    controls the camera's vertical angle, where:
    - 0° = horizontal (forward)
    - -90° = nadir (straight down)
    - +90° = zenith (straight up)
    
    This function inverts the gimbal pitch sign to match the expected behavior
    where gimbal_pitch = -90° results in the camera pointing downward.
    
    Args:
        drone_yaw_deg: Drone yaw (heading)
        drone_pitch_deg: Drone pitch
        drone_roll_deg: Drone roll
        gimbal_yaw_deg: Gimbal yaw (relative to drone)
        gimbal_pitch_deg: Gimbal pitch (relative to drone, -90 = nadir)
        gimbal_roll_deg: Gimbal roll (relative to drone)
        
    Returns:
        3x3 rotation matrix from camera frame to world frame (ENU)
    """
    # DJI gimbal convention: pitch -90° means camera points down
    # We need to negate the gimbal pitch to match our rotation convention
    adjusted_gimbal_pitch = -gimbal_pitch_deg
    
    # Build gimbal rotation (camera to gimbal/body frame)
    R_gimbal = build_rotation_matrix(gimbal_yaw_deg, adjusted_gimbal_pitch, gimbal_roll_deg)
    
    # Build drone rotation (drone body to world frame)
    R_drone = build_rotation_matrix(drone_yaw_deg, drone_pitch_deg, drone_roll_deg)
    
    # Combined rotation: first apply gimbal, then drone
    R_world_camera = R_drone @ R_gimbal
    
    return R_world_camera


def ray_plane_intersection(
    ray_origin: np.ndarray,
    ray_direction: np.ndarray,
    plane_z: float = 0.0
) -> Optional[np.ndarray]:
    """
    Intersect a ray with a horizontal plane at height z.
    
    Ray equation: P = origin + t * direction
    Plane equation: z = plane_z
    
    Args:
        ray_origin: 3D ray origin [x, y, z]
        ray_direction: 3D ray direction (normalized)
        plane_z: Z-coordinate of the plane
        
    Returns:
        3D intersection point [x, y, z] or None if no intersection
    """
    # Check if ray is parallel to plane (direction.z ≈ 0)
    if abs(ray_direction[2]) < 1e-9:
        logger.warning("Ray is parallel to ground plane, no intersection")
        return None
    
    # Solve for t: origin.z + t * direction.z = plane_z
    t = (plane_z - ray_origin[2]) / ray_direction[2]
    
    # Check if intersection is behind the ray origin
    if t < 0:
        logger.warning(f"Ray intersection is behind origin (t={t:.2f})")
        return None
    
    # Compute intersection point
    intersection = ray_origin + t * ray_direction
    
    return intersection


def enu_to_wgs84(
    enu_x: float,
    enu_y: float,
    enu_z: float,
    lat0: float,
    lon0: float,
    h0: float
) -> Tuple[float, float, float]:
    """
    Convert ENU (East-North-Up) coordinates to WGS84 (lat, lon, alt).
    
    ENU is a local tangent plane coordinate system with origin at (lat0, lon0, h0).
    - East (x): positive to the east
    - North (y): positive to the north
    - Up (z): positive upward
    
    Args:
        enu_x: East coordinate in meters
        enu_y: North coordinate in meters
        enu_z: Up coordinate in meters
        lat0: Origin latitude in degrees
        lon0: Origin longitude in degrees
        h0: Origin altitude in meters
        
    Returns:
        Tuple of (latitude, longitude, altitude) in WGS84
    """
    # Use pyproj to convert local ENU to geodetic
    # We approximate by converting ENU displacement to lat/lon offsets
    
    # Meters per degree of latitude (approximately constant)
    meters_per_deg_lat = 111320.0
    
    # Meters per degree of longitude (varies with latitude)
    meters_per_deg_lon = 111320.0 * np.cos(np.deg2rad(lat0))
    
    # Convert ENU offsets to lat/lon
    lat = lat0 + (enu_y / meters_per_deg_lat)
    lon = lon0 + (enu_x / meters_per_deg_lon)
    alt = h0 + enu_z
    
    return lat, lon, alt


def project_pixel_to_gps(
    pixel_u: float,
    pixel_v: float,
    camera_config: CameraConfig,
    telemetry: SrtTelemetry,
    agl_offset_m: float = DEFAULT_AGL_OFFSET_M,
    ground_elev_m: float = DEFAULT_GROUND_ELEV_M
) -> Optional[Tuple[float, float, float]]:
    """
    Project a pixel coordinate to GPS (lat, lon, alt).
    
    This uses a simplified projection assuming a nadir camera (or nearly nadir).
    For more accurate results with non-nadir cameras, full 3D ray tracing would
    be needed.
    
    Args:
        pixel_u: Pixel x-coordinate
        pixel_v: Pixel y-coordinate
        camera_config: Camera configuration
        telemetry: Drone telemetry (GPS, attitude, gimbal)
        agl_offset_m: Assumed height above ground (AGL offset)
        ground_elev_m: Ground elevation (default: 0 = sea level)
        
    Returns:
        Tuple of (lat, lon, alt_est) or None if projection fails
    """
    # Validate telemetry
    if telemetry.lat is None or telemetry.lon is None or telemetry.altitude_m is None:
        logger.error(f"Frame {telemetry.frame_index}: Missing GPS data")
        return None
    
    # For now, use a simplified approach similar to the existing ROS code
    # This assumes the camera is pointing approximately nadir
    
    # Get camera FOV
    fov_rad_h = np.deg2rad(camera_config.fov_deg)
    # Compute vertical FOV (assuming similar aspect ratio)
    aspect_ratio = camera_config.image_width_px / camera_config.image_height_px
    fov_rad_v = 2 * np.arctan(np.tan(fov_rad_h / 2) / aspect_ratio)
    
    # Compute ground coverage (assuming nadir view)
    altitude_agl = telemetry.altitude_m - ground_elev_m - agl_offset_m
    ground_width_m = 2 * altitude_agl * np.tan(fov_rad_h / 2)
    ground_height_m = 2 * altitude_agl * np.tan(fov_rad_v / 2)
    
    # Ground sample distance (meters per pixel)
    gsd_x = ground_width_m / camera_config.image_width_px
    gsd_y = ground_height_m / camera_config.image_height_px
    
    # Pixel offset from image center
    center_u = camera_config.image_width_px / 2
    center_v = camera_config.image_height_px / 2
    
    offset_u = pixel_u - center_u
    offset_v = center_v - pixel_v  # Invert Y (image coordinates vs. world)
    
    # Convert to meters
    offset_east_m = offset_u * gsd_x
    offset_north_m = offset_v * gsd_y
    
    # Apply rotation based on drone/gimbal yaw
    yaw_rad = np.deg2rad(telemetry.drone_yaw_deg or 0.0)
    
    # If gimbal yaw is available, use it; otherwise use drone yaw
    if telemetry.gimbal_yaw_deg is not None:
        yaw_rad = np.deg2rad(telemetry.gimbal_yaw_deg)
    
    # Rotate offsets by yaw
    east_m = offset_east_m * np.cos(yaw_rad) - offset_north_m * np.sin(yaw_rad)
    north_m = offset_east_m * np.sin(yaw_rad) + offset_north_m * np.cos(yaw_rad)
    
    # Convert to lat/lon
    lat, lon, alt = enu_to_wgs84(
        east_m, north_m, 0.0,
        telemetry.lat, telemetry.lon, telemetry.altitude_m
    )
    
    logger.debug(f"Frame {telemetry.frame_index}: Pixel ({pixel_u:.1f}, {pixel_v:.1f}) → "
                f"GPS ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
    
    return lat, lon, alt


def project_detection_bbox_center(
    bbox_xyxy: Tuple[float, float, float, float],
    camera_config: CameraConfig,
    telemetry: SrtTelemetry,
    agl_offset_m: float = DEFAULT_AGL_OFFSET_M,
    ground_elev_m: float = DEFAULT_GROUND_ELEV_M
) -> Optional[Tuple[float, float, float, float, float]]:
    """
    Project the center of a detection bounding box to GPS.
    
    Args:
        bbox_xyxy: Bounding box in (x1, y1, x2, y2) format
        camera_config: Camera configuration
        telemetry: Drone telemetry
        agl_offset_m: AGL offset
        ground_elev_m: Ground elevation
        
    Returns:
        Tuple of (center_u, center_v, lat, lon, alt_est) or None
    """
    x1, y1, x2, y2 = bbox_xyxy
    
    # Compute bbox center
    center_u = (x1 + x2) / 2.0
    center_v = (y1 + y2) / 2.0
    
    # Project to GPS
    result = project_pixel_to_gps(
        center_u, center_v,
        camera_config,
        telemetry,
        agl_offset_m,
        ground_elev_m
    )
    
    if result is None:
        return None
    
    lat, lon, alt = result
    
    return center_u, center_v, lat, lon, alt
