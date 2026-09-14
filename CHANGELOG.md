# Changelog

## 0.2.0

### ArduPilot 4.7 support

- `get_param` / `set_param` accept both the 4.6 names (`WPNAV_*`, centimetres) and the 4.7 names (`WP_*`, SI units), translated for the connected firmware. See `zenparams.py`.
- The firmware version is read at connection (`drone.ap_version`, `drone.param_naming`).

### MAVLink relay (`GCS=True`) rewritten

- New `zenrelay.py`. Several clients per TCP port, messages routed to their target (a `COMMAND_ACK` only reaches the instance that sent the command), no more slowdown when a datagram holds several messages.
- Network errors no longer stop the relay silently (a closed UDP peer used to kill it on Windows). A TCP vehicle link reconnects automatically.
- Zenmav reads the relay in-process instead of connecting to `tcp:127.0.0.1:14550`. Port 14550 stays open for other clients.
- The `baud` argument is now used for serial links in GCS mode.
- `drone.relay.stats()`: messages received, sent and dropped per link.

### Connection

- Only the heartbeat of an autopilot is accepted as the vehicle. Heartbeats of ground stations, other Zenmav instances, gimbals and companion computers are ignored, whatever the relay or router in between.
- The MAVLink system ID stays automatic. When another system on the link already uses it, Zenmav picks a free ID between 200 and 254 instead of sharing it. Serial links now use that range.
- Zenmav sends a GCS heartbeat every second.
- The connection can be read from several threads (main script, fence monitor, callbacks) without one thread swallowing the messages of another.

### Behaviour changes to know

- **GCS failsafe now works through the relay.** Mission Planner's heartbeat reaches the vehicle. With `FS_GCS_ENABLE` set, losing the ground station during a flight now triggers the configured action (RTL, LAND...), as it would without Zenmav.
- `close_all_connections()` and `RTL()` put back the default rate of the messages Zenmav requested. A ground station that stays connected sees the usual rates again.
- Startup takes about 1.5 s longer (Zenmav looks for system IDs already in use).
- `get_global_pos`, `get_local_pos` and `get_rc_value` print a warning and request the message again every 2 s without data, instead of waiting silently. After the connection is closed they raise `ConnectionError`.
- `get_rc_value` returns `None` for a channel outside 1-18 instead of blocking. `get_global_pos(time_tag=True)` now fills `timestamp` (seconds since autopilot boot).
- `set_mode` raises `RuntimeError` when the vehicle type is unknown.
- pymavlink is switched to MAVLink 2 when Zenmav is imported.

### Removed

- `split_connections()`, `message_forwarder()`, and the `connections` and `last_message_req` attributes (internals of the previous relay).

### Tests

- `tests/`: relay and connection tests with fake MAVLink systems, no simulator needed.
- `tests/sitl/`: scenarios against ArduCopter SITL (several instances, GCS failsafe, link faults, load, README flight).
