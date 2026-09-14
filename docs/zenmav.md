# Zenmav – API Reference

This document lists the public attributes and methods of `zenmav.core.Zenmav`,
the `wp` waypoint class, the gimbal controller and the software fence, with
their signatures, purpose, parameters and return values.

```python
from zenmav.core import Zenmav
from zenmav.zenpoint import wp
```

---

## Table of Contents

1. Constructor, connection & MAVLink routing
2. Waypoints (`wp`)
3. Flight-mode helpers
4. Navigation & motion commands
5. Telemetry
6. Parameters & RC utilities
7. Autonomous scan utilities
8. CSV helpers
9. Gimbal control
10. Software fence
11. Demo manoeuvre (auto-flip)

---

## 1  Constructor, connection & MAVLink routing

### `Zenmav(ip = "tcp:127.0.0.1:5762", baud = None, gps_thresh = None, GCS = False, tcp_ports = [14551], boundary_path = None)`

Connects to the drone, waits for a heartbeat, detects the firmware version and parameter naming, reads the home position and starts the optional fence monitor.

| Argument        | Type          | Description |
| --------------- | ------------- | ----------- |
| `ip`            | str           | MAVLink connection string. SITL: `tcp:127.0.0.1:5762`. UDP: `udp:<ip>:14551`. Serial: `/dev/ttyACM0`, `COM3`… Zenith Siyi link: `udpout:192.168.144.12:19856`. |
| `baud`          | int \| None   | Baud rate for serial connections. `None` uses the pymavlink default (115200). |
| `gps_thresh`    | float \| None | Distance in metres at which a **global** waypoint is considered reached. Converted at init into latitude/longitude deltas (`lat_thresh`, `lon_thresh`) around home. Defaults to `WP_RADIUS_M` (`WPNAV_RADIUS` before ArduPilot 4.7) + 1 m. Must not be smaller than the ArduPilot waypoint radius, otherwise init blocks printing a warning. |
| `GCS`           | bool          | If `True`, Zenmav relays the drone link to TCP servers so ground stations and other scripts can connect at the same time (see below). |
| `tcp_ports`     | list[int]     | Relay TCP ports opened in addition to 14550 when `GCS=True`. Defaults to `[14551]`. |
| `boundary_path` | str \| None   | Path to a software fence `.toml` file (see section 10 and `docs/Boundary_TOML.md`). |

Attributes set at init:

| Attribute      | Description |
| -------------- | ----------- |
| `connection`   | The pymavlink connection used by Zenmav. |
| `home`         | `wp` (global frame) from `HOME_POSITION`, or the current position if home can't be read. |
| `ap_version`   | Firmware version `(major, minor, patch)`, or `None` if `AUTOPILOT_VERSION` was not received. |
| `param_naming` | `"legacy"` (ArduPilot 4.6, `WPNAV_*`) or `"si"` (ArduPilot 4.7+, `WP_*`). |
| `gimbal`       | `GimbalController` instance (section 9). |
| `limits`       | `Limits` instance, only if `boundary_path` was given (section 10). |
| `relay`        | `MavRelay` instance in GCS mode (see below), otherwise `None`. |

### MAVLink routing (`GCS=True`)

Zenmav opens the drone link given by `ip` and shares it through TCP servers on `0.0.0.0:14550` and on each port of `tcp_ports` (`zenrelay.py`). Each port accepts several clients. Zenmav itself reads the relay in-process, without a socket. Connect Mission Planner / QGroundControl or another script to `tcp:<computer_ip>:14551`.

- Messages are routed the way ArduPilot routes them: a message aimed at a system (a `COMMAND_ACK`, for instance) only goes to the link where that system was seen. Everything else is broadcast.
- Heartbeats from GCS, ADS-B and onboard controllers always reach the vehicle (ArduPilot needs the Mission Planner heartbeat for its GCS failsafe). They are not forwarded to the other clients, so that older Zenmav versions are not confused about the vehicle type.
- A client whose MAVLink system ID is already used by another client receives a `STATUSTEXT`, and Zenmav picks another ID automatically.
- Link errors (client disconnected, UDP peer gone, vehicle TCP link reset) are logged and never stop the relay. A TCP vehicle link reconnects automatically.

| Function                  | Purpose |
| ------------------------- | ------- |
| `close_all_connections()` | Restores the message rates requested by Zenmav, closes the drone link and, in GCS mode, stops the relay and its TCP servers. |
| `relay.stats()`           | Messages received, sent and dropped for each link of the relay. |

### `connect(ip_address = "tcp:127.0.0.1:5762", baud = None)`

Called by the constructor; do not call it again. Blocks until the heartbeat of an **autopilot** is received. Heartbeats from ground stations, other Zenmav instances, gimbals and companion computers are ignored. A warning is printed every 5 s while waiting.

The MAVLink system ID is chosen automatically, there is nothing to configure: the port number modulo 255 (15 in GCS mode), or a free ID between 200 and 254 when another system on the link already uses it (serial links always use that range). Zenmav then sends a GCS heartbeat every second, so that other instances see it.

The connection can be read from several threads (main script, fence monitor, callbacks): each thread receives its own copy of the message stream.

---

## 2  Waypoints (`wp`)

### `wp(x, y, z, frame = None, hdg = None, timestamp = None, name = "no name")`

Position container used by every navigation and telemetry method.

| Frame         | Attributes          | Meaning |
| ------------- | ------------------- | ------- |
| `"global"`    | `lat`, `lon`, `alt` | Degrees, degrees, metres above home. |
| `"local"`     | `N`, `E`, `D`       | Metres North, East, Down from the EKF origin (Down positive: altitude is negative). |
| `"base_link"` | `F`, `R`, `D`       | Forward, Right, Down in the drone body frame (used for velocities). |

- `coordinates`: tuple `(x, y, z)`.
- If `frame` is `None`, it is guessed: more than 4 decimals on `x` → `"global"`, otherwise `"local"`. **Always pass `frame` explicitly**, computed local values can easily have more than 4 decimals.
- `copy()` returns an independent copy, `show()` prints the coordinates.

Navigation methods also accept a plain list/tuple, interpreted in the frame of the method (`global_target` → global, `local_target` → local, `speed_target` → base_link).

---

## 3  Flight-mode helpers

| Function | Purpose |
| -------- | ------- |
| `set_mode(mode, max_retries = 3)` | Switches flight mode by name (`GUIDED`, `LOITER`, `RTL`, `ALT_HOLD`, `BRAKE`, `FLIP`, …) and checks the heartbeat to confirm. |
| `arm()` | Waits until pre-arm checks pass, sends the arm command and blocks until motors are armed. It is NOT recommended on real flights: the pilot should be the only one arming the drone. |
| `takeoff(altitude = 10, threshold = 1, while_moving = None)` | Requires GUIDED mode and armed motors. Takes off to `altitude` [m], waits until within `threshold` [m] of it, then holds directly above the takeoff point. `while_moving` is called repeatedly while climbing. |
| `guided_arm_takeoff(height = 20)` | `set_mode("GUIDED")`, `arm()` and `takeoff(height)`. |
| `RTL(while_moving = None)` | Return-to-Launch. Waits for landing and disarm, **then restores message rates and closes the connection** (the relay keeps running in GCS mode). `while_moving` is called repeatedly until landed. |

---

## 4  Navigation & motion commands

### `global_target(waypoint, while_moving = None, wait_to_reach = True, heading = None)`

Fly to a **GPS position**, altitude relative to home.

- `waypoint`: `wp` or `[lat, lon, rel_alt]`. A local `wp` is converted to global using `home`.
- `while_moving`: optional function called repeatedly while in transit.
- `wait_to_reach`: if `False`, sends the command and returns immediately. Otherwise blocks until latitude and longitude are within `gps_thresh` of the target.
- `heading`: optional heading in degrees (0 = North). If `None`, ArduPilot keeps its default yaw behaviour.

---

### `local_target(waypoint, acceptance_radius = 5, while_moving = None, turn_into_wp = False, wait_to_reach = True, heading = None)`

Fly to a **local NED** position (metres).

- `waypoint`: `wp` or `[North, East, Down]`. Down is positive: 10 m above origin is `D = -10`.
- `acceptance_radius`: 3D distance [m] at which the target is considered reached.
- `turn_into_wp`: if `True`, the drone yaws to face the target.
- `heading`: optional heading in degrees (0 = North), ignored if `turn_into_wp` is `True`.
- `while_moving`, `wait_to_reach`: same as `global_target`.

---

### `speed_target(waypoint, yaw_rate = None)`

**Body-frame velocity** command, non blocking.

- `waypoint`: `wp` (`base_link` frame) or `[forward, right, down]` in m/s. Passing a global or local `wp` raises `ValueError`.
- `yaw_rate`: optional yaw rate in deg/s.

Send it repeatedly at a steady rate (e.g. 5–10 Hz); send `[0, 0, 0]` to stop.

---

### `yaw_target(yaw_angle, max_rate = 45, relative = False, clockwise = -1)`

Rotates to `yaw_angle` degrees (0 = North, wrapped to 0–360) at up to `max_rate` deg/s. `relative=True` makes the angle an offset from the current heading. `clockwise` is currently ignored: ArduPilot picks the direction.

---

### `orbit(center, radius, speed, clockwise = True, N_turns = 1, force = False, initial_position_threshold = 0.1, radius_tolerance = 1)`

Flies `N_turns` circles of `radius` [m] around `center` at `speed` [m/s], facing the center.

- `center`: `wp`, local or global.
- Refuses to start if the centripetal acceleration `speed² / radius` exceeds 1.5 m/s², unless `force=True`.
- Moves to the closest point of the circle, temporarily setting `WP_RADIUS_M` to `initial_position_threshold / 2` (restored at the end), yaws to the center, then tracks the circle with velocity commands and a PID on the radius.
- Aborts if the drone gets more than `radius_tolerance` [m] inside the circle.

---

### `convert_to_global(local_pos, reference_point = None)`

Converts a local `wp` (N, E, D) to a global `wp` (`alt = -D`). `reference_point` is a `wp` or `(lat, lon)`, defaulting to `home`. **The `wp` passed is modified in place** and returned.

### `convert_to_local(global_pos, reference_point = None)`

Converts a global `wp` or `(lat, lon, alt)` to a new local `wp` (N, E, D) relative to `reference_point` (`wp` or `(lat, lon)`, defaults to `home`).

---

### `is_near_waypoint(actual, target, threshold = 2.0)`

Returns `True` if `actual` is close to `target`:

- global `wp`: latitude and longitude within `lat_thresh` / `lon_thresh` (from `gps_thresh`); `threshold` is ignored.
- local `wp`: 3D distance `< threshold` [m].
- numbers: `abs(actual - target) < threshold`.

---

## 5  Telemetry

| Function | Returns |
| -------- | ------- |
| `get_global_pos(time_tag = False, heading = False)` | Global `wp` (`lat`, `lon`, `alt` relative to home). With `heading=True`, `pos.hdg` holds the heading in degrees. `time_tag` currently has no effect. |
| `get_local_pos(frequency_hz = 60)` | Local `wp` (`N`, `E`, `D`) from `LOCAL_POSITION_NED`. |
| `get_attitude()` | `(roll, pitch, yaw)` in degrees, yaw in 0–360, or `None` on timeout. |
| `get_battery()` | Object with `.voltage` [V] and `.current` [A], or `None` on timeout. |
| `set_home(timeout = 2.0)` | Reads `HOME_POSITION` into `home`. Returns `True` on success. |
| `message_request(message_type, freq_hz = 10)` | Asks the autopilot to stream a message ID (e.g. `mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS`) at `freq_hz`. Skipped if this message was already requested at the same rate. Mostly useful to develop new telemetry functions. |
| `restore_message_rates()` | Puts back the default rate of every message requested by this instance. Called by `close_all_connections()` and `RTL()`. |

---

## 6  Parameters & RC utilities

| Function | Purpose |
| -------- | ------- |
| `get_param(param_name, max_retries = 10, timeout = 3)` | Reads a parameter. Returns a float, or `None` if it can't be read. |
| `set_param(param_name, value, max_retries = 5, parm_type = None)` | Writes a parameter and reads it back to verify. Returns `True` on success. |
| `download_all_params(filename = None)` | Downloads all parameters to a Mission Planner compatible `NAME,value` file. Default name: `Params_<date>_<time>.param`. |
| `get_autopilot_version(timeout = 2.0)` | Firmware version `(major, minor, patch)`, or `None`. Called at init (`ap_version`). |
| `detect_param_naming()` | Returns `"si"` if `WP_RADIUS_M` exists, `"legacy"` if `WPNAV_RADIUS` exists, else raises `RuntimeError`. Called at init (`param_naming`). |
| `get_rc_value(channel)` | Raw value (µs, ~1000–2000) of RC channel 1–18. |
| `rc_override(channel_values)` | Overrides RC channels, e.g. `{'ch3': 1500, 'ch7': 1800}`. Keys `'ch1'`..`'ch8'`, values in [1000, 2000]. Channels not passed are left to the real RC. Must be refreshed periodically (≈5–10 Hz). |

### ArduPilot 4.6 / 4.7 parameter names

ArduPilot 4.7 renamed the `WPNAV_*` parameters to `WP_*` and converted them to SI units. `get_param` and `set_param` accept both spellings on both firmwares: the name is translated for the connected firmware and values stay in the units of the name you passed. For example, `set_param("WPNAV_SPEED", 300)` writes `WP_SPD = 3.0` on 4.7, and `get_param("WP_RADIUS_M")` returns metres on 4.6.

| ArduPilot 4.6    | ArduPilot 4.7   | Conversion |
| ---------------- | --------------- | ---------- |
| `WPNAV_SPEED`    | `WP_SPD`        | cm/s → m/s |
| `WPNAV_RADIUS`   | `WP_RADIUS_M`   | cm → m |
| `WPNAV_SPEED_UP` | `WP_SPD_UP`     | cm/s → m/s |
| `WPNAV_SPEED_DN` | `WP_SPD_DN`     | cm/s → m/s |
| `WPNAV_ACCEL`    | `WP_ACC`        | cm/s² → m/s² |
| `WPNAV_ACCEL_Z`  | `WP_ACC_Z`      | cm/s² → m/s² |
| `WPNAV_ACCEL_C`  | `WP_ACC_CNR`    | cm/s² → m/s² |
| `WPNAV_JERK`     | `WP_JERK`       | name only |
| `WPNAV_TER_MARGIN` | `WP_TER_MARGIN` | name only |
| `WPNAV_RFND_USE` | `WP_RFND_USE`   | name only |

Other parameters are passed unchanged. Source of truth: `src/zenmav/zenparams.py`.

---

## 7  Autonomous scan utilities

### `rectilinear_scan(detection_width = 10, altitude = 10, scan_radius = 100, safety_margin = 0, center = None)`

Covers a disc with parallel East-West passes (lawn-mower pattern), then prints the total time.

| Argument          | Unit             | Meaning |
| ----------------- | ---------------- | ------- |
| `detection_width` | m                | Sensor footprint, used as spacing between passes. |
| `altitude`        | m                | Scan altitude above home (positive up). |
| `scan_radius`     | m                | Radius of the disc to cover. |
| `safety_margin`   | m                | Currently ignored. |
| `center`          | local `wp` or `None` | Scan centre; defaults to the current local position. |

### `spiral_scan(detection_width = 10, altitude = 10, scan_radius = 100, safety_margin = 0, center = None)`

Same idea with an Archimedean spiral (100 points, spacing `detection_width`), then prints the total time. `center` must currently be given as `[North, East]`. Not very good because of ArduPilot's waypoint threshold logic and might be removed: consider building your own pattern with `local_target` / `global_target`.

---

## 8  CSV helpers

### `insert_coordinates_to_csv(file_path, waypoint, description = True)`

Appends `waypoint.coordinates` and `waypoint.name` to `file_path`. If the file doesn't exist, it is created with a header matching the frame (`Latitude, Longitude, Altitude, Name` for global, `North, East, Down, Name` for local). `description` is unused; set `waypoint.name` instead.

---

## 9  Gimbal control

Available as `drone.gimbal`.

| Function | Purpose |
| -------- | ------- |
| `set_mode(mode)` | `"retract"`, `"neutral"`, `"mavlink_targeting"`, `"rc_targeting"`, `"gps_point"`, `"sys_id_target"` or `"home_location"` (`MAV_CMD_DO_MOUNT_CONTROL`). |
| `retract()`, `neutral()`, `mavlink_targeting()` | Shortcuts for `set_mode`. |
| `set_angle(pitch = None, yaw = None, pitch_rate = None, yaw_rate = None, flags = YAW_FOLLOW)` | Angles in degrees (pitch positive up, yaw positive clockwise, wrapped to ±180). `flags`: `GimbalController.YAW_FOLLOW` (body frame) or `GimbalController.YAW_LOCK` (earth frame). Uses `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`. |
| `point_down()`, `point_forward()` | Pitch -90° / 0°, yaw 0°. |
| `point_at_location(lat, lon, alt, frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)` | Points at a GPS location (`MAV_CMD_DO_SET_ROI_LOCATION`). |
| `stop_pointing()` | Cancels the ROI (`MAV_CMD_DO_SET_ROI_NONE`). |
| `get_status()` | Dict with last commanded `mode`, `angles`, and `health` (`"ok"` if a `GIMBAL_DEVICE_ATTITUDE_STATUS` was received, else `"unknown"`). |

---

## 10  Software fence

Enabled with `Zenmav(boundary_path="fence.toml")` and available as `drone.limits`. A background thread checks every 0.25 s whether the drone is inside the polygon and runs the configured action (BRAKE or RTL) once on breach. It does **not** replace ArduPilot's fence and is less reliable: it is meant to trigger custom actions during tests. File format: `docs/Boundary_TOML.md`.

| Function | Purpose |
| -------- | ------- |
| `check_inside(point, frame = "global")` | `True` if the `wp` is inside the fence (margin included). Use `frame="local"` for a local `wp`. |
| `visualize(ax = None, show = True, plot_home = True, plot_drone = True, title = None, save_path = None, fill_alpha = 0.10, line_width = 2.0)` | Plots the fence, home and drone position with matplotlib. Returns the Axes. |
| `start_breach_monitor(period = 1.0)` / `stop_breach_monitor(timeout = None)` | Starts / stops the monitoring thread (started automatically at init). |

---

## 11  Demo manoeuvre

### `auto_flip(initial_throttle = 1750)`

1. Waits for the Enter key.
2. Stores the current global position.
3. Switches to `ALT_HOLD` with a throttle override of `initial_throttle`, then triggers `FLIP` mode after 0.5 s.
4. After 3 s switches back to `GUIDED` and returns to the stored position.

> ⚠️ **Caution** – Only test in a safe environment with plenty of altitude.

---

## Exceptions & error handling

Zenmav functions raise standard exceptions coming from `pymavlink` (connection
errors, message decode errors, …). Methods that wait for a goal (heartbeat,
waypoint, takeoff, landing) block until it is reached and can be interrupted
with `KeyboardInterrupt`.

Telemetry getters (`get_global_pos`, `get_local_pos`, `get_rc_value`) print a
warning and request the message again every 2 s without data, instead of
waiting silently. Once the connection is closed they raise `ConnectionError`.

---

## Example one-liners

```python
from pymavlink import mavutil

# Read battery voltage
print(drone.get_battery().voltage, "V")

# Stream SYS_STATUS at 1 Hz and read it directly
drone.message_request(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
print(drone.connection.recv_match(type='SYS_STATUS', blocking=True).voltage_battery / 1000, "V")

# Change waypoint speed to 5 m/s (WPNAV_SPEED = 500 cm/s before ArduPilot 4.7, also accepted)
drone.set_param('WP_SPD', 5.0)
```
