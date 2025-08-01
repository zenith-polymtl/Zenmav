# Zenmav 0.0.5 – Full API Reference

This document lists every public attribute and function included in
`zenmav.core.Zenmav` as of version 0.0.5, with their signatures, purpose,
parameters, return values and usage notes.

---

## Table of Contents

1. Constructor & basic connection  
2. Flight-mode helpers  
3. Navigation & motion commands  
4. Telemetry & helpers  
5. Parameter & RC utilities  
6. Autonomous scan utilities  
7. CSV helpers  
8. Demo manoeuvre (auto-flip)

---

## 1  Constructor & basic connection

### `__init__(self, gps_thresh: float | None = None, ip: str = 'tcp:127.0.0.1:5762')`

Initialises the object and (optionally) pre-computes GPS accuracy thresholds.

| Argument      | Type   | Description                                                                             |
|---------------|--------|-----------------------------------------------------------------------------------------|
| `gps_thresh`  | float \| None | Radius in metres used to decide when a global waypoint is *reached*. When provided, internal latitude/longitude deltas (`lat_thresh`, `lon_thresh`) are calculated from the current GPS home position. |
| `ip`          | str    | MAVLink connection string (TCP or UDP). Defaults to SITL: `tcp:127.0.0.1:5762`.         |

If `gps_thresh` is set, `self.home` is stored as the aircraft’s current global
position and is reused by `convert_to_global`.

---

## 2  Flight-mode helpers

| Function | Purpose |
|----------|---------|
| `set_mode(mode: str)` | Switches flight mode (`GUIDED`, `AUTO`, `RTL`, `ALT_HOLD`, `FLIP`, …). |
| `arm()` | Issues an `ARM/DISARM` command and blocks until motors are armed. |
| `takeoff(altitude = 10, while_moving = None)` | Performs a guided take-off to `altitude` [m]. Optional callback `while_moving()` is executed repeatedly while climbing. |
| `connect_arm_takeoff(ip = 'tcp:127.0.0.1:5762', height = 20)` | Convenience wrapper combining `connect`, `set_mode('GUIDED')`, `arm`, and `takeoff`. |
| `RTL(while_moving = None)` | Return-to-Launch, waits for landing & disarm, then closes the link. |

---

## 3  Navigation & motion commands

### `global_target(self, wp, acceptance_radius = 8e-6, while_moving = None, wait_to_reach = True)`

Fly to an **absolute GPS waypoint**.

- `wp`: tuple `(lat, lon, rel_alt)`  
- `acceptance_radius`: ignored when `gps_thresh` was supplied at construction;
  otherwise distance in **degrees** ( ≈ 1 deg ≃ 111 km).  
- `while_moving`: optional callback executed every loop iteration.  
- `wait_to_reach`: if `False`, the call is asynchronous.

---

### `local_target(self, wp, acceptance_radius = 5, while_moving = None, turn_into_wp = False)`

Move to a **local NED** position (metres) relative to home.

- `wp`: list `[North, East, Down]` (*Down* is positive).  
- `turn_into_wp`: if `True`, the craft yaws toward the target while travelling.  

Blocks until the position is reached within `acceptance_radius` [m].

---

### `speed_target(self, wp, yaw_rate = 0)`

Continuous **body-frame velocity** command.

- `wp`: `[vx_fwd, vy_right, vz_down]` in m s⁻¹ (positive *down*).  
- `yaw_rate`: deg s⁻¹ (converted to rad s⁻¹ before transmission).  

No blocking; prints the commanded speeds each call.

---

### `convert_to_global(self, local_delta, reference_point = None)`

Translate a local **(East, North)** offset (metres) into GPS coordinates.

- `local_delta`: tuple `(east_m, north_m)`  
- `reference_point`: `[lat, lon]`. `None` → `self.home`.

Returns `[lat, lon]`.

---

### `is_near_waypoint(self, actual, target, threshold = 2., gps = False)`

Utility that evaluates proximity.

- For `gps = False`: Euclidean distance `< threshold` [m].  
- For `gps = True`: compares lat/lon against internal `lat_thresh/lon_thresh`.

---

## 4  Telemetry & helpers

| Function | Description |
|----------|-------------|
| `get_global_pos(time_tag = False, heading = False)` | Returns `(lat, lon, rel_alt)` or `(timestamp, lat, lon, rel_alt)`; `heading=True` appends `hdg`. |
| `get_local_pos(frequency_hz = 60)` | Requests/streams `LOCAL_POSITION_NED` at `frequency_hz`; returns `[N, E, D]`. |
| `message_request(message_type, freq_hz = 10)` | Internal helper: configures message stream rate, avoiding duplicate requests. |

---

## 5  Parameter & RC utilities

| Function | Purpose |
|----------|---------|
| `get_param(param_name)` | Fetches a single ArduPilot parameter (float). |
| `set_param(param_name, value)` | Sets a parameter and waits for confirmation. |
| `get_rc_value(channel)` | Reads raw RC channel (1 – 18) value (1000–2000 µs). |

---

## 6  Autonomous scan utilities

### `spiral_scan(detection_width = 10, altitude = 10, scan_radius = 100, safety_margin = 0, center = None)`

Performs a circular spiral scan.

| Argument | Unit | Meaning |
|----------|------|---------|
| `detection_width` | m | Sensor footprint used as pass spacing. |
| `altitude` | m | Relative altitude for the scan (positive up → send `-altitude` as NED Z). |
| `scan_radius` | m | Nominal radius to cover. |
| `safety_margin` | m | Extra radius added once at start. |
| `center` | NED list or `None` | Scan centre; defaults to current local pos. |

Traverses waypoints sequentially; prints total duration.

---

### `rectilinear_scan(detection_width = 10, altitude = 10, scan_radius = 100, safety_margin = 0, center = None)`

Same arguments as above, but generates a lawn-mower pattern.

---

### `generate_scan_points(scan_width = 2, radius_of_scan = 13) → (x_list, y_list)`

Static helper for rectilinear pattern generation; **does not command the
aircraft**.  
Returns two lists with local coordinates (metres).

---

## 7  CSV helpers

| Function | Description |
|----------|-------------|
| `insert_coordinates_to_csv(file_path, coordinates, desc)` | Appends `(lat, lon, desc)` to `file_path` (creates file + header if missing). |
| `append_description_to_last_line(file_path, description)` | Static helper; opens CSV, appends `description` to last data row. |

---

## 8  Demo manoeuvre

### `auto_flip(self)`

1. Waits for Enter key.  
2. Stores current global position.  
3. Switches to `ALT_HOLD`, sends throttle override, triggers `FLIP` mode after
   0.5 s.  
4. After 3 s switches back to `GUIDED` and returns to the stored position.

> ⚠️ **Caution** – Only test in a safe environment with plenty of altitude.

---

## Exceptions & error handling

Zenmav functions raise standard exceptions coming from `pymavlink` (connection
timeouts, message decode errors, …). High-level methods that loop until a goal
is reached can be interrupted with `KeyboardInterrupt`.

---

## Example one-liners

```python
# Read battery voltage at 1 Hz
drone.message_request(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)
print(drone.connection.recv_match(type='SYS_STATUS', blocking=True).voltage_battery / 1000, "V")

# Change WPNAV speed to 5 m/s
drone.set_param('WPNAV_SPEED', 500)
```