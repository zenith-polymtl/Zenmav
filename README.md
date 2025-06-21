# Zenmav: Drone Control Library

![Zenmav Logo (Placeholder)](https://via.placeholder.com/150/000000/FFFFFF?text=Zenmav)

Zenmav is a Python library designed to simplify interaction and control of MAVLink-enabled drones, such as those running ArduPilot. It provides a high-level API for common drone operations like connection, arming, takeoff, waypoint navigation (global and local), speed control, and advanced scanning patterns.

Developed by **Zenith Polytechnique Montréal**, Zenmav aims to offer a robust and easy-to-use interface for drone automation and research.

## Features

*   **Easy Connection:** Quick and reliable connection to MAVLink-enabled drones (simulated or real).
*   **Heartbeat Monitoring:** Ensures stable communication by waiting for heartbeat signals.
*   **Flight Mode Control:** Seamlessly switch between different flight modes (e.g., GUIDED, RTL).
*   **Arming & Takeoff:** Simplified commands for preparing the drone for flight and initiating takeoff.
*   **Global Position Control:** Navigate to specific GPS coordinates with adjustable acceptance radii.
*   **Local Position Control (NED):** Control drone movement using local North-East-Down coordinates.
*   **Velocity Control:** Direct control over drone speed in its body frame.
*   **Data Retrieval:** Access real-time local position, global position (GPS), and RC channel values.
*   **Automated Scanning Patterns:** Implement sophisticated spiral and rectilinear (lawnmower) search patterns for area coverage.
*   **Return To Launch (RTL):** Initiate automated return and landing at the home position.
*   **CSV Logging:** Utility functions for logging GPS coordinates and descriptions to a CSV file.

## Installation

You can install Zenmav using pip:

```bash
pip install Zenmav
```

## Usage

To use Zenmav, import the `Zenmav` class from `Zenmav.core`:

```python
from Zenmav.core import Zenmav
```

### Basic Example: Connect, Takeoff, and RTL

```python
from Zenmav.core import Zenmav
import time

# Create a Zenmav instance
# ip='tcp:127.0.0.1:5762' is for SITL (Software In The Loop) simulation using MAVProxy.
# For a real drone, it might be 'udp:YOUR_UBUNTU_IP:14551'
drone = Zenmav(ip='tcp:127.0.0.1:5762', gps=True) 

try:
    # Connect, arm, and takeoff to 10 meters
    drone.connect_arm_takeoff(height=10)
    print("Drone is at 10 meters altitude.")

    # Get current global position
    current_lat, current_lon, current_alt = drone.get_global_pos()
    print(f"Current Global Position: Latitude={current_lat}, Longitude={current_lon}, Altitude={current_alt}")

    # Move to a new global target (e.g., 5 meters North, 5 meters East)
    # These are illustrative coordinates; replace with actual desired target.
    # We can use convert_to_global to calculate relative global points
    target_delta_north = 5  # meters North
    target_delta_east = 5   # meters East
    new_global_coords = drone.convert_to_global([target_delta_east, target_delta_north], reference_point=[current_lat, current_lon])
    
    print(f"Moving to a global target: {new_global_coords[0]}, {new_global_coords[1]}, {current_alt}")
    # Target (latitude, longitude, altitude)
    # Using 8e-6 as acceptance radius is very small, typically 5-10 meters is more realistic for GPS
    drone.global_target(wp=(new_global_coords[0], new_global_coords[1], current_alt), acceptance_radius=5)
    print("Reached global target.")
    time.sleep(2) # Stay for a bit

    # Move using local NED coordinates (e.g., 5 meters North, 0 East, -10 Down (at 10m altitude))
    current_local_pos = drone.get_local_pos()
    print(f"Current Local Position: {current_local_pos}")
    # Move 5 meters North relative to current local position
    drone.local_target(wp=[current_local_pos[0] + 5, current_local_pos[1], current_local_pos[2]], acceptance_radius=3)
    print("Reached local target.")
    time.sleep(2)

    # Perform a spiral scan (e.g., 50m radius, 5m detection width, at 10m altitude)
    print("Starting spiral scan...")
    drone.spiral_scan(largeur_detection=5, altitude=10, rayon_scan=50)
    print("Spiral scan completed.")
    time.sleep(2)

    # Return to launch and land
    drone.RTL()

except Exception as e:
    print(f"An error occurred: {e}")
    # Attempt to land safely in case of an error
    try:
        drone.RTL()
    except Exception as land_e:
        print(f"Error during emergency landing: {land_e}")
```

## Zenmav Class Reference

The `Zenmav` class encapsulates all functionalities for controlling the drone.

```python
class Zenmav():
    def __init__(self, gps_thresh=None, ip='tcp:127.0.0.1:5762', gps=True):
```

### `__init__(self, gps_thresh=None, ip='tcp:127.0.0.1:5762', gps=True)`

Initializes the Zenmav object, connecting to the drone and optionally fetching its home GPS position.

*   **`gps_thresh`** (float, optional): A threshold in meters used for `is_near_waypoint` when comparing GPS coordinates. If provided, `lat_thresh` and `lon_thresh` will be calculated based on this. Defaults to `None`.
*   **`ip`** (str, optional): The IP address and port for MAVLink connection. Defaults to `'tcp:127.0.0.1:5762'` for SITL.
    *   **Simulation (MAVProxy):** `'tcp:127.0.0.1:5762'`
    *   **Real Drone (Ubuntu Companion):** `'udp:<ip_ubuntu>:14551'` (Ensure UDP communication is allowed).
*   **`gps`** (bool, optional): If `True`, the drone's current global position will be fetched and set as `self.home` during initialization. Defaults to `True`.

### `connect(self, ip_address='tcp:127.0.0.1:5762')`

Establishes a MAVLink connection to the drone and waits for a heartbeat to confirm communication.

*   **`ip_address`** (str, optional): The IP address and port for connection. Defaults to `'tcp:127.0.0.1:5762'`.

### `global_target(self, wp, acceptance_radius=8e-6, while_moving=None, wait_to_reach=True)`

Sends a movement command to the drone for a specific global GPS coordinate.

*   **`wp`** (tuple): Target waypoint as \( (\text{latitude, longitude, altitude in meters}) \).
*   **`acceptance_radius`** (float, optional): Distance in meters at which the target is considered reached. This is converted to an approximate latitude/longitude difference internally if `gps_thresh` was set during initialization. Defaults to \( 8 \times 10^{-6} \) degrees (very small, consider adjusting for real flights, e.g., `5` for 5 meters).
*   **`while_moving`** (function, optional): A Python function to execute repeatedly while the drone is in transit to the target. Defaults to `None`.
*   **`wait_to_reach`** (bool, optional): If `True`, the method blocks until the drone reaches the target waypoint. If `False`, the command is sent, and the method returns immediately. Defaults to `True`.

### `is_near_waypoint(self, actual: list, target: list, threshold: float = 2., gps=False)`

Checks if the `actual` position is within a `threshold` distance of the `target` position.

*   **`actual`** (list): Current position of the drone.
    *   If `gps` is `True`: `[latitude, longitude]`
    *   If `gps` is `False`: `[North, East, Down]` (local NED coordinates).
*   **`target`** (list): Target position to compare against. Format matches `actual`.
*   **`threshold`** (float, optional): The distance (in meters for local, or an internally calculated GPS delta if `gps_thresh` was set) within which the drone is considered "near" the target. Defaults to `2` meters.
*   **`gps`** (bool, optional): If `True`, performs a GPS-based check using `lat_thresh` and `lon_thresh` (calculated from `gps_thresh` at init). If `False`, uses Euclidean distance for local coordinates.

### `get_local_pos(self, frequency_hz=60)`

Retrieves the drone's local position in the North-East-Down (NED) coordinate system.

*   **`frequency_hz`** (int, optional): The desired frequency (in Hz) at which to request `LOCAL_POSITION_NED` messages from the drone. Defaults to `60`.
*   **Returns:** `list`: The local position as `[North (m), East (m), Down (m)]`. (Note: Down is positive, so altitude above home is negative).

### `get_global_pos(self, time_tag=False, heading=False)`

Retrieves the drone's global GPS position (Latitude, Longitude, Altitude relative to home).

*   **`time_tag`** (bool, optional): If `True`, the timestamp (time since boot in seconds) will be included in the returned tuple. Defaults to `False`.
*   **`heading`** (bool, optional): If `True`, the drone's heading (degrees from North) will be included in the returned tuple. Defaults to `False`.
*   **Returns:** `tuple`:
    *   If `time_tag=False`, `heading=False`: \( (\text{latitude (deg), longitude (deg), altitude (m)}) \)
    *   If `time_tag=True`, `heading=False`: \( (\text{timestamp (s), latitude (deg), longitude (deg), altitude (m)}) \)
    *   If `time_tag=False`, `heading=True`: \( (\text{latitude (deg), longitude (deg), altitude (m), heading (deg)}) \)
    *   If `time_tag=True`, `heading=True`: \( (\text{timestamp (s), latitude (deg), longitude (deg), altitude (m), heading (deg)}) \)

### `get_rc_value(self, channel)`

Retrieves the raw value of a specific RC (Radio Control) channel.

*   **`channel`** (int): The RC channel number (1-18) to read.
*   **Returns:** `int`: The raw RC channel value (typically 1000-2000), or `None` if the channel is not available.

### `message_request(self, message_type, freq_hz=10)`

Sends a MAVLink command to set the interval for a specific message type, controlling its reception frequency. This avoids redundant requests if the same message type and frequency are already active.

*   **`message_type`** (int): The MAVLink message ID (e.g., `mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT`).
*   **`freq_hz`** (int, optional): The desired frequency (in Hz) at which to receive the specified message. Defaults to `10`.

### `set_mode(self, mode: str)`

Sets the drone's flight mode.

*   **`mode`** (str): The name of the desired flight mode (e.g., `'GUIDED'`, `'AUTO'`, `'RTL'`).

### `arm(self)`

Arms the drone's motors. The method blocks until arming is confirmed.

### `takeoff(self, altitude=10, while_moving=None)`

Commands the drone to takeoff to a specified relative altitude. Requires the drone to be in 'GUIDED' mode and armed.

*   **`altitude`** (int, optional): The target altitude in meters, relative to the home position. Defaults to `10`.
*   **`while_moving`** (function, optional): A Python function to execute repeatedly while the drone is ascending. Defaults to `None`.

### `connect_arm_takeoff(self, ip='tcp:127.0.0.1:5762', height=20)`

A convenience method to quickly connect, set the mode to 'GUIDED', arm the motors, and initiate takeoff.

*   **`ip`** (str, optional): The IP address for MAVLink connection. Defaults to `'tcp:127.0.0.1:5762'`.
*   **`height`** (int, optional): The target takeoff altitude in meters. Defaults to `20`.

### `convert_to_global(self, local_delta: tuple, reference_point=None)`

Converts local NED coordinate deltas (North, East from a reference) into global GPS coordinates.

*   **`local_delta`** (tuple): A tuple `(east_delta_meters, north_delta_meters)` representing a displacement from the `reference_point`.
*   **`reference_point`** (list, optional): The GPS coordinates `[latitude, longitude]` from which the `local_delta` is applied. If `None`, `self.home` (established during `__init__`) is used as the reference.
*   **Returns:** `list`: The calculated global GPS coordinates `[latitude, longitude]`.

### `local_target(self, wp, acceptance_radius=5, while_moving=None, turn_into_wp=False)`

Sends a movement command to the drone using local North-East-Down (NED) coordinates relative to the home position.

*   **`wp`** (list): Target waypoint as `[North (m), East (m), Down (m)]`. Note: Down is positive, so a target altitude of 10m above home would be `-10`.
*   **`acceptance_radius`** (int, optional): The distance in meters at which the drone is considered to have reached the target. Defaults to `5`.
*   **`while_moving`** (function, optional): A Python function to execute repeatedly while the drone is in transit. Defaults to `None`.
*   **`turn_into_wp`** (bool, optional): If `True`, the drone will adjust its yaw to face the target waypoint as it approaches. Defaults to `False`.

### `speed_target(self, wp: list, yaw_rate=0)`

Sends a continuous velocity command to the drone in its body frame (Forward, Right, Down).

*   **`wp`** (list): A list `[forward_velocity (m/s), right_velocity (m/s), down_velocity (m/s)]`. Note: Down is positive, so `down_velocity` is positive for descending.
*   **`yaw_rate`** (float, optional): The desired yaw rate (rotation around the vertical axis) in degrees per second. Defaults to `0`.

### `RTL(self, while_moving=None)`

Commands the drone to perform a Return To Launch (RTL) maneuver. The method blocks until the drone has landed and its motors are disarmed, then closes the MAVLink connection.

*   **`while_moving`** (function, optional): A Python function to execute repeatedly while the drone is performing the RTL. Defaults to `None`.

### `insert_coordinates_to_csv(self, file_path, coordinates, desc)`

Inserts a single set of GPS coordinates and a description into a CSV file. If the file does not exist, it creates it with a header.

*   **`file_path`** (str): The path to the CSV file (e.g., `'flight_log.csv'`).
*   **`coordinates`** (tuple): A tuple `(latitude, longitude)` of the coordinates to insert.
*   **`desc`** (str): A string description associated with the coordinates.

### `append_description_to_last_line(file_path, description)`

Appends a description to the last data line of a CSV file in a new column. This method is provided within the class but does not use `self`; it acts as a utility function.

*   **`file_path`** (str): The path to the CSV file.
*   **`description`** (str): The description string to append.

### `spiral_scan(self, largeur_detection=10, altitude=10, rayon_scan=100, safety_margin=0, center=None)`

Generates and executes a spiral flight path to scan a circular area. The method measures and prints the total time taken for the scan.

*   **`largeur_detection`** (int, optional): The effective detection width of the drone's payload in meters. This determines the spacing between spiral passes. Defaults to `10`.
*   **`altitude`** (int, optional): The relative altitude in meters at which to perform the scan. Defaults to `10`.
*   **`rayon_scan`** (int, optional): The maximum radius of the area to be scanned in meters. Defaults to `100`.
*   **`safety_margin`** (int, optional): An additional distance in meters added to `rayon_scan` to compensate for initial positioning errors. Defaults to `0`.
*   **`center`** (list, optional): Local NED coordinates `[N, E, D]` for the center of the spiral. If `None`, the drone's current local position when the function is called is used as the center.

### `rectilinear_scan(self, largeur_detection=10, altitude=10, rayon_scan=100, safety_margin=0, center=None)`

Generates and executes a rectilinear (lawnmower) flight path to scan a circular area. The method measures and prints the total time taken for the scan.

*   **`largeur_detection`** (int, optional): The effective detection width of the drone's payload in meters. This determines the spacing between parallel lines. Defaults to `10`.
*   **`altitude`** (int, optional): The relative altitude in meters at which to perform the scan. Defaults to `10`.
*   **`rayon_scan`** (int, optional): The maximum radius of the area to be scanned in meters. Defaults to `100`.
*   **`safety_margin`** (int, optional): An additional distance in meters added to `rayon_scan` to compensate for initial positioning errors. Defaults to `0`.
*   **`center`** (list, optional): Local NED coordinates `[N, E, D]` for the center of the scan. If `None`, the drone's current local position when the function is called is used as the center.

### `generate_scan_points(scan_width=2, radius_of_scan=13)`

*Note: This helper function is defined within the class but does not utilize instance-specific data. It's intended to generate the `x` and `y` coordinates for a rectilinear scan pattern. The provided code has an indentation issue that might prevent it from always returning the expected values. It should be used for testing point generation rather than directly called for flight operations.*

*   **`scan_width`** (int, optional): The width of each scan pass. Defaults to `2`.
*   **`radius_of_scan`** (int, optional): The radius of the circular area to cover. Defaults to `13`.
*   **Returns:** `tuple`: A tuple `(x, y)` where `x` and `y` are lists of local coordinates representing the scan path.

## Contributing

We welcome contributions! If you have suggestions, bug reports, or want to contribute code, please refer to our contribution guidelines (if available).

## License

This project is licensed under the MIT License - see the `LICENSE` file for details (if available).
