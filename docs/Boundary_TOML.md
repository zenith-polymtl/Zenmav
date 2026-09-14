# Fence Config (TOML) — v1

## Overview

This file defines a **software fence** for Zenmav: a **keep-in polygon** with an optional horizontal **margin**.
When `Zenmav(boundary_path="fence.toml")` is used, a background thread reads the drone's GPS position every 0.25 s. If the drone is outside the polygon, Zenmav runs the configured `action` once (BRAKE or RTL) and stops monitoring.

This fence does **not** replace ArduPilot's own fence and is less reliable. It is meant to trigger custom behaviour during tests.

- **Format:** TOML
- **Version:** 1
- **Shapes supported (v1):** inclusion polygons only

### Frames

- **global:** vertices given as `[lat, lon]` (decimal degrees, WGS-84)
- **local:** vertices given as `[North, East]` in **metres**, relative to the drone's home position at init

---

## Minimal example (global)

```toml
version = 1
name    = "Test Site A"
frame   = "global"                  # global | local
margin  = 5.0                       # metres (pre-breach buffer)
action  = "rtl"                     # brake | rtl

[z]
min = 10.0                          # metres, validated but not enforced yet
max = 120.0

[[regions]]
kind   = "include-polygon"
latlon = [
  [45.501000, -73.567000],
  [45.502000, -73.567000],
  [45.502000, -73.565000],
  [45.501000, -73.565000]
]
```

## Minimal example (local)

```toml
version = 1
name    = "Hangar Test Box"
frame   = "local"
margin  = 2.0
action  = "brake"

[z]
min = 0.0
max = 30.0

[[regions]]
kind   = "include-polygon"
points = [                          # [North, East] metres from home
  [0.0,   0.0],
  [120.0, 0.0],
  [120.0, 60.0],
  [0.0,   60.0]
]
```

More examples in `docs/fence_configs_examples/`.

---

## Keys (top-level)

- `version` (**int**, default `1`): schema version. Must be `1`.
- `name` (**string**, optional): human-readable label.
- `frame` (**string**, default `"global"`): `"global"` or `"local"`.
- `margin` (**float**, default `0.0`): metres. The polygon is shrunk inward by this distance, so the action triggers before reaching the real border. Keep it small enough for the polygon not to disappear.
- `action` (**string**, default `"brake"`): action on breach.
  - `"brake"`: switches to BRAKE mode.
  - `"rtl"`: calls `Zenmav.RTL()`, which blocks until landing and then closes the connection.
- `[z]` (**table**, optional):
  - `min` (**float**, optional, metres)
  - `max` (**float**, optional, metres)
  - If both are present, must satisfy `min <= max`. Altitude limits are **not enforced** in v1.

Other keys (e.g. `origin`) are ignored in v1: local fences are always relative to home.

## Regions (array of tables)

Each region must be:

- `kind = "include-polygon"`

Provide coordinates depending on `frame`; only the matching key is read:

- For **global**: `latlon = [[lat, lon], ...]`
- For **local**: `points = [[north, east], ...]` in metres

### Rules

- Polygons must have **≥ 3 vertices**.
- Polygons are implicitly closed: no need to repeat the first vertex.
- Only **inclusion polygons** are supported in v1 (exclusions/holes will come later).
- All regions are validated, but **only the first region is used** as the fence in v1.

## Using the fence

```python
from zenmav.core import Zenmav

drone = Zenmav(boundary_path="docs/fence_configs_examples/parc_mont_royal.toml")

print(drone.limits.check_inside(drone.get_global_pos()))  # True if inside
drone.limits.visualize()                                  # plot fence, home and drone (matplotlib)
drone.limits.stop_breach_monitor()                        # disable the automatic action
```

## Units & conventions

- Distances are **metres**.
- `lat, lon` are decimal degrees (WGS-84).
- Global polygons are projected to the UTM zone of their mean point for distance computations.

## Common validation errors

- `version` other than `1`
- `frame` not in `{global, local}`
- `kind` other than `include-polygon`
- Missing `latlon` (global) or `points` (local) list in a region
- Fewer than 3 vertices
- Latitude outside [-90, 90] or longitude outside [-180, 180]
- `margin`, `z.min` or `z.max` not a number
- `z.min > z.max`
- No region defined
