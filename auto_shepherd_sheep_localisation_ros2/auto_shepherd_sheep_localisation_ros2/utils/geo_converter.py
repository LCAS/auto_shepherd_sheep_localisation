"""
Lightweight geographic coordinate converter for field boundary and path mapping.
Converts between GPS (lat/lon) and local XY (meters) coordinates.
"""

import math
from typing import List, Tuple


class MapConverter:
    """
    A lightweight utility to convert between GPS coordinates (lat/lon) and 
    local XY coordinates (meters) relative to a field boundary center.
    """
    
    def __init__(self, map_coords_latlon: List[Tuple[float, float]]):
        """
        Initialize converter with boundary coordinates.
        
        Args:
            map_coords_latlon: List of (latitude, longitude) tuples defining field boundary
        """
        if not map_coords_latlon:
            raise ValueError("map_coords_latlon cannot be empty")
        
        # Calculate bounding box
        lats = [c[0] for c in map_coords_latlon]
        lons = [c[1] for c in map_coords_latlon]
        
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)
        
        # Use center as origin
        self.origin_lat = (min_lat + max_lat) / 2.0
        self.origin_lon = (min_lon + max_lon) / 2.0
        
        # Store for reference
        self.min_lat = min_lat
        self.max_lat = max_lat
        self.min_lon = min_lon
        self.max_lon = max_lon
    
    def latlon_to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        """
        Convert GPS coordinates to local XY coordinates (meters).
        
        Args:
            lat: Latitude
            lon: Longitude
            
        Returns:
            (x_meters, y_meters) tuple where:
            - x increases eastward (positive longitude direction)
            - y increases northward (positive latitude direction)
        """
        # Approximate conversion: 1 degree latitude ≈ 111,320 meters
        lat_meters_per_deg = 111320.0
        
        # Longitude varies by latitude
        lon_meters_per_deg = 111320.0 * math.cos(math.radians(self.origin_lat))
        
        # Calculate offset from origin in meters
        y_meters = (lat - self.origin_lat) * lat_meters_per_deg
        x_meters = (lon - self.origin_lon) * lon_meters_per_deg
        
        return x_meters, y_meters
    
    def xy_to_latlon(self, y_meters: float, x_meters: float) -> Tuple[float, float]:
        """
        Convert local XY coordinates (meters) to GPS coordinates.
        
        Args:
            y_meters: North-south offset in meters from origin
            x_meters: East-west offset in meters from origin
            
        Returns:
            (latitude, longitude) tuple
        """
        # Reverse of latlon_to_xy
        lat_meters_per_deg = 111320.0
        lon_meters_per_deg = 111320.0 * math.cos(math.radians(self.origin_lat))
        
        lat = self.origin_lat + (y_meters / lat_meters_per_deg)
        lon = self.origin_lon + (x_meters / lon_meters_per_deg)
        
        return lat, lon
