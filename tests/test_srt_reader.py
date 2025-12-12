"""
Tests for SRT parsing functionality.
"""
import pytest
from datetime import timedelta
from pathlib import Path
import tempfile

from sheepgeo.io.srt_reader import SrtReader, SrtTelemetry, apply_fallback_defaults


def test_srt_telemetry_model():
    """Test SrtTelemetry dataclass."""
    telemetry = SrtTelemetry(
        frame_index=0,
        timestamp=timedelta(seconds=0),
        lat=52.0,
        lon=-1.0,
        altitude_m=50.0,
        drone_yaw_deg=45.0,
        drone_pitch_deg=0.0,
        drone_roll_deg=0.0,
        gimbal_yaw_deg=0.0,
        gimbal_pitch_deg=-90.0,
        gimbal_roll_deg=0.0
    )
    
    assert telemetry.frame_index == 0
    assert telemetry.lat == 52.0
    assert telemetry.lon == -1.0
    assert telemetry.altitude_m == 50.0


def test_parse_sample_srt_format1():
    """Test parsing DJI SRT format 1 (GPS in parentheses)."""
    srt_content = """1
00:00:00,000 --> 00:00:00,033
GPS (52.5074, -1.1278, 50.2) drone_yaw: 45.2 drone_pitch: -10.5 drone_roll: 2.1 gimbal_yaw: 0.0 gimbal_pitch: -90.0 gimbal_roll: 0.0 ISO: 100

2
00:00:00,033 --> 00:00:00,066
GPS (52.5075, -1.1279, 50.3) drone_yaw: 45.5 drone_pitch: -10.3 drone_roll: 2.0 gimbal_yaw: 0.0 gimbal_pitch: -90.0 gimbal_roll: 0.0 ISO: 100
"""
    
    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.srt', delete=False) as f:
        f.write(srt_content)
        temp_path = Path(f.name)
    
    try:
        reader = SrtReader(temp_path)
        telemetry_list = reader.parse_all()
        
        assert len(telemetry_list) == 2
        
        # Check first frame
        telem0 = telemetry_list[0]
        assert telem0.frame_index == 0
        assert telem0.lat == pytest.approx(52.5074)
        assert telem0.lon == pytest.approx(-1.1278)
        assert telem0.altitude_m == pytest.approx(50.2)
        assert telem0.drone_yaw_deg == pytest.approx(45.2)
        assert telem0.drone_pitch_deg == pytest.approx(-10.5)
        assert telem0.drone_roll_deg == pytest.approx(2.1)
        assert telem0.gimbal_yaw_deg == pytest.approx(0.0)
        assert telem0.gimbal_pitch_deg == pytest.approx(-90.0)
        assert telem0.gimbal_roll_deg == pytest.approx(0.0)
        assert telem0.iso == 100
        
        # Check second frame
        telem1 = telemetry_list[1]
        assert telem1.frame_index == 1
        assert telem1.lat == pytest.approx(52.5075)
        assert telem1.lon == pytest.approx(-1.1279)
        assert telem1.altitude_m == pytest.approx(50.3)
        
    finally:
        temp_path.unlink()


def test_parse_sample_srt_format2():
    """Test parsing DJI SRT format 2 (bracketed format)."""
    srt_content = """1
00:00:00,000 --> 00:00:00,033
[latitude: 52.5074] [longitude: -1.1278] [rel_alt: 50.2] [yaw: 45.2] [pitch: -10.5] [roll: 2.1] [gimbal_yaw: 0.0] [gimbal_pitch: -90.0] [gimbal_roll: 0.0]

2
00:00:00,033 --> 00:00:00,066
[latitude: 52.5075] [longitude: -1.1279] [rel_alt: 50.3] [yaw: 45.5] [pitch: -10.3] [roll: 2.0] [gimbal_yaw: 0.0] [gimbal_pitch: -90.0] [gimbal_roll: 0.0]
"""
    
    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.srt', delete=False) as f:
        f.write(srt_content)
        temp_path = Path(f.name)
    
    try:
        reader = SrtReader(temp_path)
        telemetry_list = reader.parse_all()
        
        assert len(telemetry_list) == 2
        
        # Check first frame
        telem0 = telemetry_list[0]
        assert telem0.lat == pytest.approx(52.5074)
        assert telem0.lon == pytest.approx(-1.1278)
        assert telem0.altitude_m == pytest.approx(50.2)
        assert telem0.drone_yaw_deg == pytest.approx(45.2)
        
    finally:
        temp_path.unlink()


def test_parse_missing_fields():
    """Test parsing SRT with missing fields."""
    srt_content = """1
00:00:00,000 --> 00:00:00,033
GPS (52.5074, -1.1278, 50.2) drone_yaw: 45.2

2
00:00:00,033 --> 00:00:00,066
GPS (52.5075, -1.1279, 50.3) drone_yaw: 45.5 gimbal_pitch: -90.0
"""
    
    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.srt', delete=False) as f:
        f.write(srt_content)
        temp_path = Path(f.name)
    
    try:
        reader = SrtReader(temp_path)
        telemetry_list = reader.parse_all()
        
        assert len(telemetry_list) == 2
        
        # First frame should have some missing fields
        telem0 = telemetry_list[0]
        assert telem0.lat == pytest.approx(52.5074)
        assert telem0.drone_yaw_deg == pytest.approx(45.2)
        # pitch, roll, gimbal angles should be None initially
        # but might be interpolated or filled with fallbacks
        
    finally:
        temp_path.unlink()


def test_apply_fallback_defaults():
    """Test fallback defaults for missing telemetry."""
    telemetry = SrtTelemetry(
        frame_index=0,
        timestamp=timedelta(seconds=0),
        lat=52.0,
        lon=-1.0,
        altitude_m=50.0,
        drone_yaw_deg=45.0
    )
    
    # Apply fallback defaults
    telemetry = apply_fallback_defaults(telemetry)
    
    # Check that fallbacks were applied
    assert telemetry.gimbal_pitch_deg == -90.0  # nadir
    assert telemetry.gimbal_roll_deg == 0.0
    assert telemetry.gimbal_yaw_deg == 45.0  # uses drone yaw
    assert telemetry.drone_pitch_deg == 0.0
    assert telemetry.drone_roll_deg == 0.0


def test_interpolation():
    """Test interpolation of missing values."""
    srt_content = """1
00:00:00,000 --> 00:00:00,033
GPS (52.5000, -1.0000, 50.0) drone_yaw: 0.0

2
00:00:00,033 --> 00:00:00,066
GPS (52.5000, -1.0000, 50.0)

3
00:00:00,066 --> 00:00:00,100
GPS (52.5000, -1.0000, 50.0) drone_yaw: 90.0
"""
    
    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='.srt', delete=False) as f:
        f.write(srt_content)
        temp_path = Path(f.name)
    
    try:
        reader = SrtReader(temp_path)
        telemetry_list = reader.parse_all()
        
        assert len(telemetry_list) == 3
        
        # Middle frame should have interpolated yaw
        telem1 = telemetry_list[1]
        assert telem1.drone_yaw_deg is not None
        # Should be approximately 45.0 (midpoint between 0 and 90)
        assert 40.0 <= telem1.drone_yaw_deg <= 50.0
        
    finally:
        temp_path.unlink()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
