# Fence Config (TOML) — v1

## Overview

This file defines **software fences** for Zenmav. A fence is a set of **keep‑in polygons** with optional altitude limits and a horizontal **margin**. When enabled, Zenmav clamps or rejects setpoints that would leave the allowed area, and can trigger actions (e.g., brake/RTL).

- **Format:** TOML
- **Version:** 1
- **Shapes supported (v1):** inclusion polygons only

### Frames

- **global:** vertices given as `lat, lon` (degrees, WGS‑84)
- **local:** vertices given as `x, y` in **meters**, in a local NED plane anchored at `origin`

---

## Minimal example (global)

```toml
version = 1
name    = "Test Site A"
frame   = "global"                  # global | local
margin  = 5.0
action = "rtl"                       # meters (pre-breach buffer)

[z]
min = 10.0                          # meters AMSL/AGL (interpretation up to usage), useless as of now
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
action = "brake"

[z]
min = 0.0
max = 30.0

[[regions]]
kind   = "include-polygon"
points = [
  [0.0,   0.0],
  [120.0, 0.0],
  [120.0, 60.0],
  [0.0,   60.0]
]
```

---

## Keys (top‑level)

- `version` (**int**, required): schema version. Must be `1`.
- `name` (**string**, optional): human‑readable label.
- `frame` (**string**, required): `"global"` or `"local"`.
- `origin` (**array | string**, optional): for `local` frames, `"home"` or `[lat, lon, alt]`.Ignored for `global`, but allowed (future projection features).
- `margin` (**float**, default `0.0`): horizontal pre‑breach buffer in meters.
- `action` (**string**, default `brake`): Action to take when fence is breached, rtl and brake possible as of now
- `[z]` (**table**, optional):
  - `min` (**float**, optional, meters)
  - `max` (**float**, optional, meters)
  - If both present, must satisfy `min <= max`.

## Regions (array of tables)

Each region must be:

- `kind = "include-polygon"`

Provide coordinates depending on frame:

- For **global**: `latlon = [[lat, lon], ...]`
- For **local** : `points = [[x, y], ...]` in meters

### Rules

- Polygons must have **≥ 3 vertices**.
- Do **not** repeat the first vertex at the end; polygons are implicitly closed.
- Mixed coordinate lists (e.g., `latlon` together with `points`) are **invalid**.
- Only **inclusion polygons** are supported in v1 (exclusions/holes will come later).

## Units & conventions

- Distances are **meters**.
- Speeds (if used elsewhere) are **m/s**.
- `lat, lon` are decimal degrees (WGS‑84).
- Altitude reference (AMSL vs AGL) is up to your runtime policy; the file just stores numbers.

## Common validation errors

- `frame` not in `{global, local}`
- Missing `latlon` (global) or `points` (local) per region
- Fewer than 3 vertices
- `z.min > z.max`
- Unknown `kind` value
