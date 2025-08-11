from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Literal, Dict, Any, Union
from pathlib import Path
from shapely.geometry import Point, Polygon
from shapely.prepared import prep
from pyproj import Transformer
import threading

try:
    import tomllib  # Python 3.11+
except ModuleNotFoundError:
    import tomli as tomllib  # Python 3.10 fallback

LatLon = Tuple[float, float]
XY = Tuple[float, float]


@dataclass
class PolygonRegion:
    include: Literal[True] = True
    frame: Literal["global", "local"] = "global"
    latlon: Optional[List[LatLon]] = None  # used when frame == "global"
    points: Optional[List[XY]] = None  # used when frame == "local"


@dataclass
class FenceSpec:
    version: int
    name: Optional[str] = None
    frame: Literal["global", "local"] = "global"
    origin: Union[str, Tuple[float, float, float], None] = None
    margin_m: float = 0.0
    action: str = "brake"  # "brake" or "rtl"
    z_min_m: Optional[float] = None
    z_max_m: Optional[float] = None
    regions: List[PolygonRegion] = field(default_factory=list)

    def validate(self) -> None:
        if self.version != 1:
            raise ValueError(f"Unsupported fence version: {self.version}")
        if self.frame not in ("global", "local"):
            raise ValueError(f"frame must be 'global' or 'local', got {self.frame}")
        if (
            self.z_min_m is not None
            and self.z_max_m is not None
            and self.z_min_m > self.z_max_m
        ):
            raise ValueError(f"z.min ({self.z_min_m}) > z.max ({self.z_max_m})")
        if not self.regions:
            raise ValueError("at least one region is required")
        for r in self.regions:
            if r.frame != self.frame:
                raise ValueError("all regions must match top-level frame")
            if r.latlon is None and r.points is None:
                raise ValueError(
                    "region must define 'latlon' (global) or 'points' (local)"
                )
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
    name = data.get("name")
    frame = str(data.get("frame", "global")).lower()

    margin_m = _as_float(data.get("margin", 0.0), "margin")

    action = str(data.get("action", "brake")).lower()

    ztbl = data.get("z", {})
    z_min_m = _as_float(ztbl.get("min"), "z.min") if isinstance(ztbl, dict) else None
    z_max_m = _as_float(ztbl.get("max"), "z.max") if isinstance(ztbl, dict) else None

    regions_raw = data.get("regions", [])
    regions: List[PolygonRegion] = []
    for i, r in enumerate(regions_raw, start=1):
        kind = str(r.get("kind", "")).lower()
        if kind != "include-polygon":
            raise ValueError(
                f"regions[{i}] kind must be 'include-polygon' in v1, got {kind!r}"
            )
        reg = PolygonRegion(include=True, frame=frame)
        if frame == "global":
            ll = r.get("latlon")
            if not isinstance(ll, list):
                raise ValueError(
                    f"regions[{i}] must define 'latlon' list for frame=global"
                )
            reg.latlon = [(float(a), float(b)) for a, b in ll]
        else:  # local
            pts = r.get("points")
            if not isinstance(pts, list):
                raise ValueError(
                    f"regions[{i}] must define 'points' list for frame=local"
                )
            reg.points = [(float(x), float(y)) for x, y in pts]
        regions.append(reg)

    spec = FenceSpec(
        version=version,
        name=name,
        frame=frame,
        margin_m=float(margin_m),
        action=action,
        z_min_m=z_min_m,
        z_max_m=z_max_m,
        regions=regions,
    )
    spec.validate()
    return spec


class Limits:
    def __init__(self, drone, config_path, check_interval=0.25):
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

        self.fence_spec = load_fence_toml(config_path)

        if self.fence_spec.frame == "global":
            poly_ll = self.fence_spec.regions[0].latlon  # list[(lat,lon)]
            self.prepared, self.transformer = self.build_fence_global(
                poly_ll, margin_m=self.fence_spec.margin_m
            )
        else:
            poly_xy = self.fence_spec.regions[0].points  # list[(x,y)] meters
            self.prepared, self.transformer = self.build_fence_local(
                poly_xy, margin_m=self.fence_spec.margin_m
            )

        if self.fence_spec.frame == "global":
            print(
                f"Loaded fence spec: {self.fence_spec.regions[0].latlon}, {self.fence_spec.regions[0].frame}"
            )
        else:
            print(
                f"Loaded fence spec: {self.fence_spec.regions[0].points}, {self.fence_spec.regions[0].frame}"
            )

        self.start_breach_monitor(period=check_interval)

    def start_breach_monitor(self, period: float = 1.0):
        if getattr(self, "_breach_thread", None) and self._breach_thread.is_alive():
            return  # already running
        self._breach_stop = threading.Event()
        self._breach_thread = threading.Thread(
            target=self._breach_loop, args=(period,), daemon=True
        )
        self._breach_thread.start()

    def stop_breach_monitor(self, timeout: float | None = None):
        if getattr(self, "_breach_stop", None):
            self._breach_stop.set()
        # Only join if we’re NOT the breach thread
        if (
            getattr(self, "_breach_thread", None)
            and threading.current_thread() is not self._breach_thread
        ):
            self._breach_thread.join(timeout=timeout)

    def breach_action(self):
        if self.fence_spec.action == "brake":
            print("Executing fence breach action: BRAKE")
            self.drone.set_mode("BRAKE")
        elif self.fence_spec.action == "rtl":
            print("Executing fence breach action: RTL")
            self.drone.set_mode("RTL")
        else:
            print(f"Unknown fence action: {self.fence_spec.action}")

        self.stop_breach_monitor()

    def _breach_loop(self, period: float):
        while not self._breach_stop.is_set():
            try:
                pos = self.drone.get_global_pos()
                inside = self.check_inside(pos, frame="global")
                if not inside:
                    print(f"WARNING: Drone is outside the fence at position {pos}.")
                    # Consider thread-safety here if your MAVLink senders aren’t thread-safe
                    self.breach_action()
            except Exception as e:
                print(f"[fence] breach loop error: {e}")
            # Use wait() so we can stop promptly
            self._breach_stop.wait(period)

    def _utm_crs_from_lonlat(self, lon: float, lat: float) -> str:
        zone = int((lon + 180) // 6) + 1
        return f"EPSG:{32600 + zone if lat >= 0 else 32700 + zone}"  # 326xx=N, 327xx=S

    def build_fence_global(self, polygon_ll, margin_m: float = 0.0):
        """
        polygon_ll: list[(lat, lon), ...]
        margin_m:   shrink inside by this margin (>=0). Use buffer(-margin).
        Returns: (prepared_polygon_in_meters, transformer)
        """
        # Mean point of zone as center of UTM projection
        latc = sum(lat for lat, _ in polygon_ll) / len(polygon_ll)
        lonc = sum(lon for _, lon in polygon_ll) / len(polygon_ll)
        transformer = Transformer.from_crs(
            "EPSG:4326", self._utm_crs_from_lonlat(lonc, latc), always_xy=True
        )

        # project polygon to meters (x=Easting, y=Northing)
        poly_xy = Polygon([transformer.transform(lon, lat) for lat, lon in polygon_ll])
        if margin_m:
            poly_xy = poly_xy.buffer(-float(margin_m))  # negative = shrink inward
        return prep(poly_xy), transformer

    def build_fence_local(self, polygon_xy, margin_m: float = 0.0):
        """polygon_xy: list[(x=East, y=North), ...] in meters"""
        poly = Polygon([(e, n) for (n, e) in polygon_xy])
        if margin_m:
            shrunk = poly.buffer(-float(margin_m))  # shrink inward by margin
            if not shrunk.is_empty:
                poly = shrunk

        latc = self.home[0]
        lonc = self.home[1]
        transformer = Transformer.from_crs(
            "EPSG:4326", self._utm_crs_from_lonlat(lonc, latc), always_xy=True
        )
        self.origin_xy = transformer.transform(lonc, latc)
        return prep(poly), transformer  # no transformer needed

    def check_inside(self, point, *, frame="global"):
        """
        point:
        global: (lat, lon) or {'lat':..,'lon':..}
        local : (N, E) meters or {'north':..,'east':..}
        frame: "global" if point is lat/lon, "local" if point is N/E in meters
        """
        if frame == "global":
            lat, lon = float(point[0]), float(point[1])
            xg, yg = self.transformer.transform(lon, lat)

            # NEW: if fence is local, express the point in local NED meters by subtracting origin
            if self.fence_spec.frame == "local":
                x0, y0 = self.origin_xy
                x, y = xg - x0, yg - y0
            else:
                x, y = xg, yg
        else:
            # Local NED point: map (N,E) -> (y,x)
            n, e = float(point[0]), float(point[1])
            x, y = e, n

        return self.prepared.context.covers(Point(x, y))
