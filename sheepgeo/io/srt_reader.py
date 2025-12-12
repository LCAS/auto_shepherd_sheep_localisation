"""
DJI SRT subtitle parser for extracting drone telemetry.

Parses DJI drone subtitle files (.SRT) which contain per-frame telemetry including:
- GPS coordinates (latitude, longitude, altitude)
- Drone attitude (yaw, pitch, roll)
- Gimbal attitude (yaw, pitch, roll)
- Camera settings (ISO, shutter speed, etc.)
"""
import re
import logging
from datetime import datetime, timedelta
from typing import List, Optional, Dict, Any
from pathlib import Path

import pysrt
from pydantic import BaseModel, Field, field_validator


logger = logging.getLogger(__name__)


class SrtTelemetry(BaseModel):
    """Telemetry data extracted from a single SRT subtitle entry."""
    
    # Timing
    frame_index: int
    timestamp: timedelta
    
    # GPS
    lat: Optional[float] = None
    lon: Optional[float] = None
    altitude_m: Optional[float] = None
    
    # Drone attitude (degrees)
    drone_yaw_deg: Optional[float] = None
    drone_pitch_deg: Optional[float] = None
    drone_roll_deg: Optional[float] = None
    
    # Gimbal attitude (degrees)
    gimbal_yaw_deg: Optional[float] = None
    gimbal_pitch_deg: Optional[float] = None
    gimbal_roll_deg: Optional[float] = None
    
    # Camera settings (optional)
    iso: Optional[int] = None
    shutter_speed: Optional[str] = None
    fnum: Optional[float] = None
    ev: Optional[float] = None
    
    # Additional metadata
    focal_length_mm: Optional[float] = None
    color_temp: Optional[int] = None
    
    class Config:
        frozen = False  # Allow modifications for interpolation


class SrtReader:
    """
    Parse DJI SRT files to extract per-frame telemetry.
    
    DJI drones (Mavic Air 2, Mini 2 SE, etc.) embed telemetry in SRT subtitle
    files alongside video recordings. This parser extracts GPS, attitude,
    and gimbal data for geo-referencing detections.
    """
    
    def __init__(self, srt_path: Path):
        """
        Initialize the SRT reader.
        
        Args:
            srt_path: Path to the .SRT file
        """
        self.srt_path = Path(srt_path)
        self.subtitles = pysrt.open(str(self.srt_path), encoding='utf-8')
        logger.info(f"Loaded {len(self.subtitles)} subtitle entries from {srt_path}")
    
    def parse_all(self) -> List[SrtTelemetry]:
        """
        Parse all subtitle entries and return telemetry list.
        
        Returns:
            List of SrtTelemetry objects, one per frame
        """
        telemetry_list = []
        
        for idx, subtitle in enumerate(self.subtitles):
            try:
                telemetry = self._parse_subtitle(subtitle, idx)
                telemetry_list.append(telemetry)
            except Exception as e:
                logger.warning(f"Failed to parse subtitle {idx}: {e}")
                # Create a minimal telemetry entry with just timing
                telemetry = SrtTelemetry(
                    frame_index=idx,
                    timestamp=subtitle.start.to_time()
                )
                telemetry_list.append(telemetry)
        
        # Fill missing values via interpolation
        telemetry_list = self._interpolate_missing(telemetry_list)
        
        return telemetry_list
    
    def _parse_subtitle(self, subtitle: pysrt.SubRipItem, frame_index: int) -> SrtTelemetry:
        """
        Parse a single subtitle entry.
        
        Args:
            subtitle: pysrt SubRipItem
            frame_index: Frame index
            
        Returns:
            SrtTelemetry object
        """
        text = subtitle.text
        
        # Initialize telemetry
        telemetry = SrtTelemetry(
            frame_index=frame_index,
            timestamp=timedelta(
                hours=subtitle.start.hours,
                minutes=subtitle.start.minutes,
                seconds=subtitle.start.seconds,
                milliseconds=subtitle.start.milliseconds
            )
        )
        
        # Parse GPS coordinates
        # Format examples:
        # "GPS (51.5074, -0.1278, 50.2)"
        # "[latitude: 51.5074] [longtitude: -0.1278] [rel_alt: 50.200 abs_alt: 100.5]"
        gps_match = re.search(r'GPS\s*\(([-+]?\d+\.\d+),\s*([-+]?\d+\.\d+),\s*([-+]?\d+\.?\d*)\)', text)
        if gps_match:
            telemetry.lat = float(gps_match.group(1))
            telemetry.lon = float(gps_match.group(2))
            telemetry.altitude_m = float(gps_match.group(3))
        else:
            # Alternative format
            lat_match = re.search(r'\[latitude:\s*([-+]?\d+\.\d+)\]', text, re.IGNORECASE)
            lon_match = re.search(r'\[long?itude:\s*([-+]?\d+\.\d+)\]', text, re.IGNORECASE)
            alt_match = re.search(r'\[(?:rel_)?alt(?:itude)?:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
            
            if lat_match:
                telemetry.lat = float(lat_match.group(1))
            if lon_match:
                telemetry.lon = float(lon_match.group(1))
            if alt_match:
                telemetry.altitude_m = float(alt_match.group(1))
        
        # Parse drone attitude (yaw, pitch, roll)
        # Format: "drone_yaw: 45.2, drone_pitch: -10.5, drone_roll: 2.1"
        # Or: "[yaw: 45.2] [pitch: -10.5] [roll: 2.1]"
        yaw_match = re.search(r'(?:drone_)?yaw:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
        pitch_match = re.search(r'(?:drone_)?pitch:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
        roll_match = re.search(r'(?:drone_)?roll:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
        
        if yaw_match:
            telemetry.drone_yaw_deg = float(yaw_match.group(1))
        if pitch_match:
            telemetry.drone_pitch_deg = float(pitch_match.group(1))
        if roll_match:
            telemetry.drone_roll_deg = float(roll_match.group(1))
        
        # Parse gimbal attitude
        # Format: "gimbal_yaw: 0.0, gimbal_pitch: -90.0, gimbal_roll: 0.0"
        gimbal_yaw_match = re.search(r'gimbal[_\s]?yaw:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
        gimbal_pitch_match = re.search(r'gimbal[_\s]?pitch:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
        gimbal_roll_match = re.search(r'gimbal[_\s]?roll:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
        
        if gimbal_yaw_match:
            telemetry.gimbal_yaw_deg = float(gimbal_yaw_match.group(1))
        if gimbal_pitch_match:
            telemetry.gimbal_pitch_deg = float(gimbal_pitch_match.group(1))
        if gimbal_roll_match:
            telemetry.gimbal_roll_deg = float(gimbal_roll_match.group(1))
        
        # Parse camera settings
        iso_match = re.search(r'ISO:\s*(\d+)', text, re.IGNORECASE)
        shutter_match = re.search(r'Shutter:\s*([\d/]+)', text, re.IGNORECASE)
        fnum_match = re.search(r'[fF]num:\s*([\d.]+)', text, re.IGNORECASE)
        ev_match = re.search(r'EV:\s*([-+]?\d+\.?\d*)', text, re.IGNORECASE)
        
        if iso_match:
            telemetry.iso = int(iso_match.group(1))
        if shutter_match:
            telemetry.shutter_speed = shutter_match.group(1)
        if fnum_match:
            telemetry.fnum = float(fnum_match.group(1))
        if ev_match:
            telemetry.ev = float(ev_match.group(1))
        
        return telemetry
    
    def _interpolate_missing(self, telemetry_list: List[SrtTelemetry]) -> List[SrtTelemetry]:
        """
        Interpolate missing telemetry values over time.
        
        This is useful when some frames have incomplete data. We perform
        linear interpolation for numeric fields.
        
        Args:
            telemetry_list: List of telemetry objects
            
        Returns:
            List with interpolated values
        """
        if len(telemetry_list) < 2:
            return telemetry_list
        
        # Fields to interpolate
        numeric_fields = [
            'lat', 'lon', 'altitude_m',
            'drone_yaw_deg', 'drone_pitch_deg', 'drone_roll_deg',
            'gimbal_yaw_deg', 'gimbal_pitch_deg', 'gimbal_roll_deg'
        ]
        
        for field in numeric_fields:
            # Find indices with valid values
            valid_indices = []
            valid_values = []
            
            for i, telem in enumerate(telemetry_list):
                value = getattr(telem, field)
                if value is not None:
                    valid_indices.append(i)
                    valid_values.append(value)
            
            # Skip if no valid values or only one valid value
            if len(valid_indices) < 2:
                continue
            
            # Interpolate missing values
            for i in range(len(telemetry_list)):
                if getattr(telemetry_list[i], field) is None:
                    # Find surrounding valid values
                    prev_idx = None
                    next_idx = None
                    
                    for vi in valid_indices:
                        if vi < i:
                            prev_idx = vi
                        elif vi > i and next_idx is None:
                            next_idx = vi
                            break
                    
                    # Linear interpolation
                    if prev_idx is not None and next_idx is not None:
                        prev_val = valid_values[valid_indices.index(prev_idx)]
                        next_val = valid_values[valid_indices.index(next_idx)]
                        weight = (i - prev_idx) / (next_idx - prev_idx)
                        interpolated = prev_val + weight * (next_val - prev_val)
                        setattr(telemetry_list[i], field, interpolated)
                    elif prev_idx is not None:
                        # Use previous value
                        setattr(telemetry_list[i], field, valid_values[valid_indices.index(prev_idx)])
                    elif next_idx is not None:
                        # Use next value
                        setattr(telemetry_list[i], field, valid_values[valid_indices.index(next_idx)])
        
        return telemetry_list
    
    def get_telemetry_at_frame(self, frame_index: int) -> Optional[SrtTelemetry]:
        """
        Get telemetry for a specific frame index.
        
        Args:
            frame_index: Frame index
            
        Returns:
            SrtTelemetry or None if not available
        """
        if frame_index < 0 or frame_index >= len(self.subtitles):
            return None
        
        return self._parse_subtitle(self.subtitles[frame_index], frame_index)


def apply_fallback_defaults(telemetry: SrtTelemetry) -> SrtTelemetry:
    """
    Apply fallback defaults for missing telemetry values.
    
    If gimbal or attitude angles are missing, we assume:
    - Gimbal pitch ≈ -90° (nadir)
    - Gimbal roll ≈ 0°
    - Gimbal yaw ≈ drone heading
    
    Args:
        telemetry: SrtTelemetry object
        
    Returns:
        Updated telemetry with fallback values
    """
    # Fallback for gimbal pitch (assume nadir)
    if telemetry.gimbal_pitch_deg is None:
        telemetry.gimbal_pitch_deg = -90.0
        logger.debug(f"Frame {telemetry.frame_index}: Using fallback gimbal pitch = -90°")
    
    # Fallback for gimbal roll
    if telemetry.gimbal_roll_deg is None:
        telemetry.gimbal_roll_deg = 0.0
        logger.debug(f"Frame {telemetry.frame_index}: Using fallback gimbal roll = 0°")
    
    # Fallback for gimbal yaw (use drone yaw)
    if telemetry.gimbal_yaw_deg is None and telemetry.drone_yaw_deg is not None:
        telemetry.gimbal_yaw_deg = telemetry.drone_yaw_deg
        logger.debug(f"Frame {telemetry.frame_index}: Using drone yaw for gimbal yaw")
    
    # Fallback for drone attitude
    if telemetry.drone_pitch_deg is None:
        telemetry.drone_pitch_deg = 0.0
        logger.debug(f"Frame {telemetry.frame_index}: Using fallback drone pitch = 0°")
    
    if telemetry.drone_roll_deg is None:
        telemetry.drone_roll_deg = 0.0
        logger.debug(f"Frame {telemetry.frame_index}: Using fallback drone roll = 0°")
    
    return telemetry
