from __future__ import annotations
from pymavlink import mavutil
import time
import numpy as np
from geopy.distance import distance
from geopy import Point
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Literal, Dict, Any, Union
from pathlib import Path
from geopy.distance import geodesic
from shapely.geometry import Point, Polygon
from shapely.prepared import prep
from pyproj import Transformer

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    import tomli as tomllib  # Python 3.10 fallback

LatLon = Tuple[float, float]
XY     = Tuple[float, float]

@dataclass
class PolygonRegion:
    include: Literal[True] = True
    frame:   Literal["global","local"] = "global"
    latlon: Optional[List[LatLon]] = None   # used when frame == "global"
    points: Optional[List[XY]] = None       # used when frame == "local"

@dataclass
class FenceSpec:
    version: int
    name: Optional[str] = None
    frame: Literal["global","local"] = "global"
    origin: Union[str, Tuple[float,float,float], None] = None
    margin_m: float = 0.0
    z_min_m: Optional[float] = None
    z_max_m: Optional[float] = None
    regions: List[PolygonRegion] = field(default_factory=list)

    def validate(self) -> None:
        if self.version != 1:
            raise ValueError(f"Unsupported fence version: {self.version}")
        if self.frame not in ("global","local"):
            raise ValueError(f"frame must be 'global' or 'local', got {self.frame}")
        if self.frame == "local":
            if self.origin is None:
                raise ValueError("local frame requires 'origin' ('home' or [lat, lon, alt])")
            if isinstance(self.origin, (list, tuple)) and len(self.origin) != 3:
                raise ValueError("origin must be [lat, lon, alt] when array is used")
        if self.z_min_m is not None and self.z_max_m is not None and self.z_min_m > self.z_max_m:
            raise ValueError(f"z.min ({self.z_min_m}) > z.max ({self.z_max_m})")
        if not self.regions:
            raise ValueError("at least one region is required")
        for r in self.regions:
            if r.frame != self.frame:
                raise ValueError("all regions must match top-level frame")
            if r.latlon is None and r.points is None:
                raise ValueError("region must define 'latlon' (global) or 'points' (local)")
            pts = r.latlon if self.frame == "global" else r.points
            if not pts or len(pts) < 3:
                raise ValueError("polygon must have at least 3 vertices")
            # light sanity check on coords
            if self.frame == "global":
                for (lat, lon) in pts:
                    if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
                        raise ValueError(f"invalid lat/lon: {lat}, {lon}")

def _as_float(x: Any, keypath: str) -> Optional[float]:
    if x is None:
        return None
    try:
        return float(x)
    except Exception as e:
        raise ValueError(f"'{keypath}' must be a number, got {x!r}") from e

def load_fence_toml(path: str | Path) -> FenceSpec:
    """Load and validate a v1 fence TOML."""
    p = Path(path)
    data = tomllib.loads(p.read_text(encoding="utf-8"))

    version = int(data.get("version", 1))
    name    = data.get("name")
    frame   = str(data.get("frame", "global")).lower()
    origin  = data.get("origin")
    if isinstance(origin, list):
        if len(origin) != 3:
            raise ValueError("origin must be [lat, lon, alt] when array is used")
        origin = (float(origin[0]), float(origin[1]), float(origin[2]))
    elif origin is not None:
        origin = str(origin)  # e.g., "home"

    margin_m = _as_float(data.get("margin", 0.0), "margin")

    ztbl = data.get("z", {})
    z_min_m = _as_float(ztbl.get("min"), "z.min") if isinstance(ztbl, dict) else None
    z_max_m = _as_float(ztbl.get("max"), "z.max") if isinstance(ztbl, dict) else None

    regions_raw = data.get("regions", [])
    regions: List[PolygonRegion] = []
    for i, r in enumerate(regions_raw, start=1):
        kind = str(r.get("kind","")).lower()
        if kind != "include-polygon":
            raise ValueError(f"regions[{i}] kind must be 'include-polygon' in v1, got {kind!r}")
        reg = PolygonRegion(include=True, frame=frame)
        if frame == "global":
            ll = r.get("latlon")
            if not isinstance(ll, list):
                raise ValueError(f"regions[{i}] must define 'latlon' list for frame=global")
            reg.latlon = [(float(a), float(b)) for a, b in ll]
        else:  # local
            pts = r.get("points")
            if not isinstance(pts, list):
                raise ValueError(f"regions[{i}] must define 'points' list for frame=local")
            reg.points = [(float(x), float(y)) for x, y in pts]
        regions.append(reg)

    spec = FenceSpec(
        version=version,
        name=name,
        frame=frame, origin=origin,
        margin_m=float(margin_m),
        z_min_m=z_min_m, z_max_m=z_max_m,
        regions=regions,
    )
    spec.validate()
    return spec



class fence():
    def __init__(self, drone, config_path):
        """
        Initialize a fence with a center point and radius.
        
        Args:
            lat (float): Latitude of the center point in degrees.
            lon (float): Longitude of the center point in degrees.
            radius (float): Radius of the fence in meters.
        """
        self.drone = drone
        self.home = drone.home
        self.config_path = config_path
        self.fence_spec = None

        self.fence_spec = load_fence_toml(config_path)

        print(f"Loaded fence spec: {self.fence_spec.regions[0].latlon}, {self.fence_spec.regions[0].frame}")


    def _utm_crs_from_lonlat(self, lon: float, lat: float) -> str:
        zone = int((lon + 180) // 6) + 1
        return f"EPSG:{32600 + zone if lat >= 0 else 32700 + zone}"  # 326xx=N, 327xx=S

    def build_fence_global(self, polygon_ll, margin_m: float = 0.0):
        """
        polygon_ll: list[(lat, lon), ...]
        margin_m:   shrink inside by this margin (>=0). Use buffer(-margin).
        Returns: (prepared_polygon_in_meters, transformer)
        """
        # choose a sensible local projection (UTM of centroid)
        latc = sum(lat for lat, _ in polygon_ll) / len(polygon_ll)
        lonc = sum(lon for _, lon in polygon_ll) / len(polygon_ll)
        transformer = Transformer.from_crs("EPSG:4326", self._utm_crs_from_lonlat(lonc, latc), always_xy=True)

        # project polygon to meters (x=Easting, y=Northing)
        poly_xy = Polygon([transformer.transform(lon, lat) for lat, lon in polygon_ll])
        if margin_m:
            poly_xy = poly_xy.buffer(-float(margin_m))  # negative = shrink inward
        return prep(poly_xy), transformer

    def inside_fence(self, point, prepared_poly, transformer, *, frame="global"):
        """
        point:
        global: (lat, lon) or {'lat':..,'lon':..}
        local : (N, E) meters or {'north':..,'east':..}
        frame: "global" if point is lat/lon, "local" if point is N/E in meters
        """
        if frame == "global":
            if isinstance(point, dict):
                lat, lon = float(point["lat"]), float(point["lon"])
            else:
                lat, lon = float(point[0]), float(point[1])
            x, y = transformer.transform(lon, lat)  # lon,lat -> x,y meters
        else:
            # Local NED point: map (N,E) -> (y,x)
            n, e = float(point[0]), float(point[1])
            x, y = e, n

        return prepared_poly.contains(Point(x, y))
