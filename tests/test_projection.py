"""
Tests for projection math (pixel to GPS conversion).
"""
import pytest
import numpy as np
from datetime import timedelta

from sheepgeo.config import CameraConfig
from sheepgeo.geo.projection import (
    pixel_to_camera_ray,
    build_rotation_matrix,
    compose_camera_rotation,
    ray_plane_intersection,
    enu_to_wgs84,
    project_pixel_to_gps
)
from sheepgeo.io.srt_reader import SrtTelemetry


def test_pixel_to_camera_ray():
    """Test pixel to camera ray conversion."""
    # Simple camera matrix (image center at 320x240, focal length 400)
    K = np.array([
        [400, 0, 320],
        [0, 400, 240],
        [0, 0, 1]
    ], dtype=np.float64)
    
    # Test center pixel
    ray = pixel_to_camera_ray(320, 240, K)
    
    # Center pixel should give a ray pointing forward (z-direction)
    assert ray[0] == pytest.approx(0.0, abs=1e-6)
    assert ray[1] == pytest.approx(0.0, abs=1e-6)
    assert ray[2] == pytest.approx(1.0, abs=1e-6)
    
    # Test that ray is normalized
    assert np.linalg.norm(ray) == pytest.approx(1.0)


def test_build_rotation_matrix():
    """Test rotation matrix construction."""
    # Identity rotation (no rotation)
    R = build_rotation_matrix(0, 0, 0)
    assert np.allclose(R, np.eye(3))
    
    # Rotation around Z axis (yaw)
    R = build_rotation_matrix(90, 0, 0)
    # After 90° yaw, X-axis should point in Y direction
    x_axis = R @ np.array([1, 0, 0])
    assert x_axis[0] == pytest.approx(0.0, abs=1e-6)
    assert x_axis[1] == pytest.approx(1.0, abs=1e-6)
    
    # Check that rotation matrix is orthogonal
    assert np.allclose(R @ R.T, np.eye(3))
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_compose_camera_rotation():
    """Test composition of drone and gimbal rotations."""
    # Drone facing north (yaw=0), gimbal nadir (pitch=-90)
    R = compose_camera_rotation(
        drone_yaw_deg=0,
        drone_pitch_deg=0,
        drone_roll_deg=0,
        gimbal_yaw_deg=0,
        gimbal_pitch_deg=-90,
        gimbal_roll_deg=0
    )
    
    # Camera should be pointing downward (negative Z in world frame)
    # Camera's Z-axis (forward) should map to world -Z
    camera_z = np.array([0, 0, 1])
    world_z = R @ camera_z
    
    # After gimbal pitch -90°, camera forward should point down
    assert world_z[2] < 0  # pointing downward
    
    # Check orthogonality
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-6)


def test_ray_plane_intersection():
    """Test ray-plane intersection."""
    # Ray from (0, 0, 10) pointing down
    ray_origin = np.array([0, 0, 10])
    ray_direction = np.array([0, 0, -1])
    
    # Intersect with ground plane at z=0
    intersection = ray_plane_intersection(ray_origin, ray_direction, plane_z=0.0)
    
    assert intersection is not None
    assert intersection[0] == pytest.approx(0.0)
    assert intersection[1] == pytest.approx(0.0)
    assert intersection[2] == pytest.approx(0.0)
    
    # Ray at an angle
    ray_direction = np.array([1, 0, -1])
    ray_direction = ray_direction / np.linalg.norm(ray_direction)
    
    intersection = ray_plane_intersection(ray_origin, ray_direction, plane_z=0.0)
    
    assert intersection is not None
    assert intersection[0] == pytest.approx(10.0)  # moved 10m east
    assert intersection[2] == pytest.approx(0.0)


def test_ray_plane_intersection_parallel():
    """Test ray-plane intersection with parallel ray."""
    # Ray parallel to ground (no Z component)
    ray_origin = np.array([0, 0, 10])
    ray_direction = np.array([1, 0, 0])
    
    intersection = ray_plane_intersection(ray_origin, ray_direction, plane_z=0.0)
    
    # Should return None (no intersection)
    assert intersection is None


def test_enu_to_wgs84():
    """Test ENU to WGS84 conversion."""
    # Origin at a known location
    lat0, lon0, h0 = 52.0, -1.0, 100.0
    
    # Test zero displacement
    lat, lon, alt = enu_to_wgs84(0, 0, 0, lat0, lon0, h0)
    
    assert lat == pytest.approx(lat0)
    assert lon == pytest.approx(lon0)
    assert alt == pytest.approx(h0)
    
    # Test displacement east (positive X)
    lat, lon, alt = enu_to_wgs84(111.32, 0, 0, lat0, lon0, h0)
    
    # Should move approximately 0.001° east
    assert lat == pytest.approx(lat0, abs=1e-6)
    assert lon > lon0
    
    # Test displacement north (positive Y)
    lat, lon, alt = enu_to_wgs84(0, 111.32, 0, lat0, lon0, h0)
    
    # Should move approximately 0.001° north
    assert lat > lat0
    assert lon == pytest.approx(lon0, abs=1e-5)


def test_project_pixel_to_gps_nadir():
    """Test pixel-to-GPS projection with nadir camera (looking straight down)."""
    # Create a simple camera config
    camera_config = CameraConfig(
        name="Test Camera",
        fov_deg=84.0,
        focal_length_mm=4.5,
        sensor_width_mm=6.4,
        sensor_height_mm=4.8,
        image_width_px=1920,
        image_height_px=1080,
    )
    
    # Create telemetry (drone at 50m altitude, looking straight down)
    telemetry = SrtTelemetry(
        frame_index=0,
        timestamp=timedelta(seconds=0),
        lat=52.0,
        lon=-1.0,
        altitude_m=50.0,
        drone_yaw_deg=0.0,  # facing north
        drone_pitch_deg=0.0,
        drone_roll_deg=0.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-90.0,  # nadir
        gimbal_roll_deg=0.0
    )
    
    # Project center pixel (should be directly below drone)
    result = project_pixel_to_gps(
        960, 540,  # center pixel
        camera_config,
        telemetry,
        agl_offset_m=0.0,  # assume drone altitude is AGL
        ground_elev_m=0.0
    )
    
    assert result is not None
    lat, lon, alt = result
    
    # Center pixel should be very close to drone GPS position
    assert lat == pytest.approx(telemetry.lat, abs=0.001)
    assert lon == pytest.approx(telemetry.lon, abs=0.001)


def test_project_pixel_to_gps_offset():
    """Test pixel-to-GPS projection with off-center pixel."""
    # Create a simple camera config
    camera_config = CameraConfig(
        name="Test Camera",
        fov_deg=84.0,
        focal_length_mm=4.5,
        sensor_width_mm=6.4,
        sensor_height_mm=4.8,
        image_width_px=1920,
        image_height_px=1080,
    )
    
    # Create telemetry
    telemetry = SrtTelemetry(
        frame_index=0,
        timestamp=timedelta(seconds=0),
        lat=52.0,
        lon=-1.0,
        altitude_m=50.0,
        drone_yaw_deg=0.0,
        drone_pitch_deg=0.0,
        drone_roll_deg=0.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-90.0,
        gimbal_roll_deg=0.0
    )
    
    # Project a pixel to the right of center
    result_right = project_pixel_to_gps(
        1440, 540,  # 1/4 from right edge
        camera_config,
        telemetry,
        agl_offset_m=0.0,
        ground_elev_m=0.0
    )
    
    # Project a pixel to the left of center
    result_left = project_pixel_to_gps(
        480, 540,  # 1/4 from left edge
        camera_config,
        telemetry,
        agl_offset_m=0.0,
        ground_elev_m=0.0
    )
    
    assert result_right is not None
    assert result_left is not None
    
    # Right pixel should have greater longitude (more east)
    assert result_right[1] > telemetry.lon
    
    # Left pixel should have smaller longitude (more west)
    assert result_left[1] < telemetry.lon


def test_project_pixel_to_gps_missing_data():
    """Test projection with missing telemetry data."""
    camera_config = CameraConfig(
        name="Test Camera",
        fov_deg=84.0,
        focal_length_mm=4.5,
        sensor_width_mm=6.4,
        sensor_height_mm=4.8,
        image_width_px=1920,
        image_height_px=1080,
    )
    
    # Telemetry with missing GPS
    telemetry = SrtTelemetry(
        frame_index=0,
        timestamp=timedelta(seconds=0),
        drone_yaw_deg=0.0,
        gimbal_pitch_deg=-90.0
    )
    
    result = project_pixel_to_gps(
        960, 540,
        camera_config,
        telemetry
    )
    
    # Should return None due to missing GPS data
    assert result is None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
