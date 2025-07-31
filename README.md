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

Below are three concise, copy-paste-ready examples that showcase different parts of Zenmav’s API.  
Each script follows the same basic pattern—connect, arm, fly, land—while using a different feature set. 
Here are a few examples:

------------------------------------------------------------------------------------------------------------------------
Example 1 — Fly to a local waypoint and come back
------------------------------------------------------------------------------------------------------------------------

```python
# example_local_target.py
# Demonstrates local_target()    (requires zenmav 0.0.5)

import time
from zenmav.core import Zenmav

def main() -> None:
    drone = Zenmav()                 # connect (default SITL TCP link)
    drone.arm()
    drone.set_mode("GUIDED")
    drone.takeoff(altitude=15)       # climb to 15 m AGL

    # Fly 30 m North, 20 m East, keep same altitude (Down = 0)
    print("Navigating to local waypoint …")
    drone.local_target([30, 20, 0])

    # Hold position 5 s
    time.sleep(5)

    # Return-to-Launch (waits for landing & disarm)
    drone.RTL()

if __name__ == "__main__":
    main()
```

------------------------------------------------------------------------------------------------------------------------
Example 2 — Quick spiral scan of a 50 m radius area
------------------------------------------------------------------------------------------------------------------------

```python
# example_spiral_scan.py
# Demonstrates spiral_scan()     (requires zenmav 0.0.5)

from zenmav.core import Zenmav

def main() -> None:
    drone = Zenmav(gps_thresh=2)     # enables waypoint-reach detection in metres
    drone.arm()
    drone.set_mode("GUIDED")
    drone.takeoff(altitude=25)       # climb to 25 m AGL

    print("Starting spiral scan …")
    drone.spiral_scan(
        largeur_detection=8,         # 8 m sensor footprint
        altitude=25,                 # keep current altitude
        rayon_scan=50,               # cover a 50 m radius
        safety_margin=10             # add 10 m buffer
    )

    drone.RTL()

if __name__ == "__main__":
    main()
```

------------------------------------------------------------------------------------------------------------------------
Example 3 — Log a GPS point to CSV and adjust cruise speed
------------------------------------------------------------------------------------------------------------------------

```python
# example_csv_and_params.py
# Shows get_global_pos(), insert_coordinates_to_csv(), set_param()   (zenmav 0.0.5)

import csv
from pathlib import Path
from zenmav.core import Zenmav

CSV_FILE = Path("waypoints.csv")

def main() -> None:
    drone = Zenmav(ip="/dev/ttyACM0") # example connection to pixhawk via USB
    drone.arm()
    drone.set_mode("GUIDED")
    drone.takeoff(altitude=10)

    # Grab current location and write it to CSV
    lat, lon, rel_alt = drone.get_global_pos()
    desc = "Take-off point"
    drone.insert_coordinates_to_csv(CSV_FILE, (lat, lon), desc)
    print(f"Saved home waypoint to {CSV_FILE}")

    # Slow the aircraft to 3 m/s (ArduPilot expects cm/s)
    drone.set_param("WPNAV_SPEED", 300)
    print("WPNAV_SPEED set to 3 m/s")

    # Fly forward at 3 m/s for ~3 s using body-frame velocity
    for _ in range(30):
        drone.speed_target([3, 0, 0])     # 3 m/s forward, level flight
    print("Short cruise complete")

    drone.RTL()

    # Optional: show CSV content
    print("\nCSV content:")
    with CSV_FILE.open() as fp:
        print(fp.read())

if __name__ == "__main__":
    main()
```

Each script is intentionally minimal:
- The constructor opens the MAVLink link, so no explicit `connect()` call is needed.
- `RTL()` blocks until touchdown, ensuring a clean shutdown.
- All examples work unchanged on SITL; simply run them with  
  `python example_name.py`.

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