# Zenmav – Drone Control Library

Zenmav is a high-level Python library that makes it easy to control drones
running ArduPilot through the MAVLink protocol.  
Developed at **Zenith Polytechnique Montréal**, it offers a clean API for
connection, flight management, autonomous navigation and data acquisition in
both simulation (SITL) and on real vehicles.

## Key Features

- **Effortless Connection** – one-line TCP/UDP connection with heartbeat check
- **Flexible Flight Modes** – switch between `GUIDED`, `AUTO`, `RTL`, `ALT_HOLD`,
  `FLIP`, …
- **Automated Pre-Flight** – arm and take off with a single call
- **Precise Navigation**
  - Global GPS waypoints with user-defined accuracy
  - Local NED waypoints relative to home
  - Real-time body-frame speed control
- **Autonomous Area Scans**
  - Spiral pattern
  - Rectilinear/lawn-mower pattern
- **Return-To-Launch** – safe RTL, auto-landing and disarm
- **Live Telemetry** – local position, global GPS, heading, RC channels
- **Parameter Management** – read/write any ArduPilot parameter
- **CSV Logging Utilities**
- **Advanced Maneuvers** – built-in automated flip demo

---

## Installation

```bash
pip install zenmav
```

SITL users only need MAVProxy running on port 5762 (default shown below).

---

## Quick Start

```python
from zenmav.core import Zenmav
import time

# 1 – Create the object and connect (simulation).
#     If you want reliable GPS accuracy checks,
#     pass gps_thresh = desired_radius_in_meters.
drone = Zenmav(ip='tcp:127.0.0.1:5762', gps_thresh=2.0)

try:
    # 2 – Arm and take off
    drone.connect_arm_takeoff(height=10)

    # 3 – Navigate to a global offset (+10 m N, +5 m E)
    lat0, lon0, alt0 = drone.get_global_pos()
    target_lat, target_lon = drone.convert_to_global(
        local_delta=(5, 10),              # (East, North) in metres
        reference_point=[lat0, lon0]
    )
    drone.global_target((target_lat, target_lon, alt0))
    
    # 4 – Move 5 m north in the local frame
    N, E, D = drone.get_local_pos()
    drone.local_target([N + 5, E, D], acceptance_radius=3)

    # 5 – Perform a spiral scan of 50 m radius
    drone.spiral_scan(largeur_detection=5, altitude=10, rayon_scan=50)

    # 6 – Return home and land
    drone.RTL()

finally:
    # Always close the link when you are done
    if getattr(drone, "connection", None):
        drone.connection.close()
```

---

## Zenmav API Reference

### Full Docs

Available in docs/zenmav.md

### Constructor

```python
class Zenmav:
    def __init__(self, gps_thresh: float | None = None,
                 ip: str = 'tcp:127.0.0.1:5762'):
```

Arguments  
- `gps_thresh` – radius (metres) used to evaluate GPS-waypoint proximity.  
  If provided the library computes internal `lat_thresh` and `lon_thresh`
  from the current home position.  
  When `gps_thresh` is not supplied, global-waypoint functions still work but
  rely on the raw acceptance_radius you pass each time.
- `ip` – MAVLink connection string:  
  - SITL: `tcp:127.0.0.1:5762` (default)  
  - Real drone (companion computer): `udp:<COMPANION_IP>:14551`

> NOTE  If `gps_thresh` is supplied, `self.home` is
> automatically set to the current GPS location.  
> If you plan to call `convert_to_global` without passing
> `reference_point`, make sure you started Zenmav with `gps_thresh`
> so that `self.home` exists.

### Connection helpers

- `connect(ip_address='tcp:127.0.0.1:5762')`
- `set_mode(mode: str)`
- `arm()`
- `takeoff(altitude=10, while_moving=None)`
- `connect_arm_takeoff(ip, height)`

### Navigation

- `global_target(wp, acceptance_radius=8e-6, while_moving=None,
  wait_to_reach=True)`
- `local_target(wp, acceptance_radius=5, while_moving=None,
  turn_into_wp=False)`
- `speed_target(wp, yaw_rate=0)`

### Position helpers

- `get_global_pos(time_tag=False, heading=False)`
- `get_local_pos(frequency_hz=60)`
- `is_near_waypoint(actual, target, threshold=2., gps=False)`
- `convert_to_global(local_delta, reference_point=None)`

### Parameter & RC utilities

- `get_param(param_name)`
- `set_param(param_name, value)`
- `get_rc_value(channel)`
- `message_request(message_type, freq_hz=10)`

### Autonomy & Scanning

- `spiral_scan(largeur_detection=10, altitude=10,
  rayon_scan=100, safety_margin=0, center=None)`
- `rectilinear_scan(largeur_detection=10, altitude=10,
  rayon_scan=100, safety_margin=0, center=None)`
- `generate_scan_points(scan_width=2, radius_of_scan=13)`  
  (static helper – returns `(x, y)` lists)

### Mission termination

- `RTL(while_moving=None)`

### CSV helpers

- `insert_coordinates_to_csv(file_path, coordinates, desc)`
- `append_description_to_last_line(file_path, description)`  
  (static helper)

### Demo maneuver

- `auto_flip()` – switches to `ALT_HOLD`, performs a flip, then
  returns to `GUIDED` and flies back to the initial location.

---

## Contributing

Issues and pull requests are welcome!  
Please fork the repository, create a feature branch and open an MR when ready.

---