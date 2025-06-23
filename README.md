# Zenmav: Drone Control Library

Zenmav is a high-level Python library designed to streamline the control and interaction with drones running Ardupilot. It provides a robust and intuitive API for essential drone operations, from connection and flight management to advanced autonomous missions and data acquisition.

Developed by **Zenith Polytechnique Montréal**, Zenmav aims to empower developers and researchers with a powerful tool for drone automation and experimental applications.

## Features

*   **Effortless Connection:** Establish stable MAVLink connections to both simulated (SITL) and physical drones.
*   **Reliable Communication:** Built-in heartbeat monitoring ensures continuous and secure communication.
*   **Flexible Flight Control:** Seamlessly transition between various flight modes (e.g., GUIDED, AUTO, RTL).
*   **Automated Pre-Flight:** Simplified commands for arming motors and executing controlled takeoffs.
*   **Precise Navigation:**
    *   **Global Waypoints:** Navigate to specific GPS coordinates with customizable acceptance criteria.
    *   **Local Waypoints:** Control drone movement in the North-East-Down (NED) coordinate system, relative to home.
    *   **Velocity Control:** Direct real-time velocity commands in the drone's body frame.
*   **Real-time Data Access:** Retrieve essential flight data, including local position, global GPS coordinates, and RC channel values.
*   **Advanced Scanning Patterns:** Implement efficient spiral and rectilinear (lawnmower) flight patterns for comprehensive area coverage.
*   **Safe Return:** Execute automated Return-To-Launch (RTL) procedures, ensuring safe landing and disarming.
*   **Integrated Data Logging:** Utility functions for logging GPS coordinates and mission descriptions to CSV files.

## Installation

Install Zenmav easily using pip:

```bash
pip install Zenmav
```

## Usage

To begin, import the `Zenmav` class from `zenmav.core`:

```python
from zenmav.core import Zenmav
```

### Quick Start Example

This example demonstrates connecting to a simulated drone, taking off, navigating to global and local waypoints, performing a scan, and returning to launch.

```python
from zenmav.core import Zenmav
import time

# Initialize Zenmav.
# 'ip' for SITL (MAVProxy): 'tcp:127.0.0.1:5762'
# For a real drone (e.g., connected via companion computer): 'udp:<IP_OF_COMPANION_COMPUTER>:14551'
# 'gps_thresh' (e.g., 2.0 meters) is crucial for accurate GPS waypoint checks.
# If gps_thresh is not set, global_target's acceptance_radius will not function as expected for GPS.
drone = Zenmav(ip='tcp:127.0.0.1:5762', gps_thresh=2.0, gps=True) 

try:
    # Connect, arm the motors, and takeoff to 10 meters altitude.
    print("Connecting, arming, and taking off...")
    drone.connect_arm_takeoff(height=10)
    print("Drone is airborne at 10 meters altitude.")
    time.sleep(1) # Allow some stability

    # Get current global position (Latitude, Longitude, Altitude relative to home).
    current_lat, current_lon, current_alt = drone.get_global_pos()
    print(f"Current Global Position: Lat={current_lat:.6f}°, Lon={current_lon:.6f}°, Alt={current_alt:.2f}m")

    # Example: Move 10 meters North and 5 meters East from the current global position.
    # The convert_to_global method helps calculate the new GPS coordinates.
    delta_north_meters = 10
    delta_east_meters = 5
    new_global_coords = drone.convert_to_global(
        local_delta=[delta_east_meters, delta_north_meters], # (East, North)
        reference_point=[current_lat, current_lon]
    )
    
    # Navigate to the new global target.
    # The 'acceptance_radius' here will use the 'gps_thresh' set during initialization for GPS accuracy.
    print(f"Navigating to global target: Lat={new_global_coords[0]:.6f}°, Lon={new_global_coords[1]:.6f}° at Alt={current_alt:.2f}m")
    drone.global_target(wp=(new_global_coords[0], new_global_coords[1], current_alt))
    print("Reached global target.")
    time.sleep(2) # Hold position

    # Get current local position (North, East, Down relative to home).
    current_local_pos = drone.get_local_pos()
    print(f"Current Local Position: North={current_local_pos[0]:.2f}m, East={current_local_pos[1]:.2f}m, Down={current_local_pos[2]:.2f}m")

    # Example: Move 5 meters North from the current local position.
    # Note: Local Z (Down) is positive, so altitude is negative.
    target_local_north = current_local_pos[0] + 5
    target_local_east = current_local_pos[1]
    target_local_down = current_local_pos[2] # Maintain current altitude
    
    print(f"Navigating to local target: North={target_local_north:.2f}m, East={target_local_east:.2f}m, Down={target_local_down:.2f}m")
    drone.local_target(wp=[target_local_north, target_local_east, target_local_down], acceptance_radius=3)
    print("Reached local target.")
    time.sleep(2)

    # Perform a spiral scan over a 50m radius with a 5m detection overlap, at 10m altitude.
    print("Initiating spiral scan...")
    drone.spiral_scan(largeur_detection=5, altitude=10, rayon_scan=50)
    print("Spiral scan completed.")
    time.sleep(2)

    # Return to launch (RTL) and land.
    print("Initiating Return To Launch (RTL)...")
    drone.RTL()
    print("Drone landed and disarmed. Mission complete.")

except Exception as e:
    print(f"\nAn error occurred during flight: {e}")
    # Attempt to perform an emergency RTL in case of an unhandled error.
    try:
        print("Attempting emergency RTL...")
        drone.RTL()
    except Exception as land_e:
        print(f"Error during emergency landing attempt: {land_e}")
finally:
    # Ensure the connection is closed even if RTL failed or wasn't called.
    if hasattr(drone, 'connection') and drone.connection is not None:
        print("Ensuring connection is closed.")
        drone.connection.close()
```

## Zenmav Class Reference

The `Zenmav` class is the primary interface for controlling the drone.

```python
class Zenmav():
    def __init__(self, gps_thresh=None, ip='tcp:127.0.0.1:5762', gps=True):
```

### `__init__(self, gps_thresh=None, ip='tcp:127.0.0.1:5762', gps=True)`

Initializes the Zenmav object, establishes connection parameters, and optionally sets the drone's home GPS position.

*   **`gps_thresh`** (`float`, optional): Defines a threshold in meters for determining if the drone is "near" a global GPS waypoint. If provided, internal `lat_thresh` and `lon_thresh` values are calculated. **Highly recommended for reliable GPS waypoint navigation.** Defaults to `None`.
*   **`ip`** (`str`, optional): The MAVLink connection string.
    *   For **SITL (MAVProxy)** simulation: `'tcp:127.0.0.1:5762'`
    *   For **real drone** connected via companion computer (e.g., Raspberry Pi on Ubuntu): `'udp:<ip_of_companion_computer>:14551'` (ensure network configuration allows UDP communication).
    *   Defaults to `'tcp:127.0.0.1:5762'`.
*   **`gps`** (`bool`, optional): If `True`, the drone's current global position is fetched upon initialization and stored as `self.home`. Defaults to `True`.

### `connect(self, ip_address='tcp:127.0.0.1:5762')`

Establishes the MAVLink connection and waits for a heartbeat signal from the drone to confirm communication.

*   **`ip_address`** (`str`, optional): The IP address and port for connection. Defaults to `'tcp:127.0.0.1:5762'`.

### `global_target(self, wp, acceptance_radius=8e-6, while_moving=None, wait_to_reach=True)`

Commands the drone to move to a specified global GPS coordinate.

*   **`wp`** (`tuple`): The target waypoint as \( (\text{latitude, longitude, altitude in meters relative to home}) \).
*   **`acceptance_radius`** (`float`, optional): **Note:** This parameter is overridden if `gps_thresh` was provided during `Zenmav` initialization. If `gps_thresh` is set, the actual acceptance criteria for latitude and longitude will be derived from it. The default value of \( 8 \times 10^{-6} \) degrees is very small and typically not practical for real-world GPS accuracy unless `gps_thresh` is configured.
*   **`while_moving`** (`callable`, optional): A function to execute repeatedly while the drone is en route to the target. Defaults to `None`.
*   **`wait_to_reach`** (`bool`, optional): If `True`, the method blocks until the drone reaches the target. If `False`, the command is sent, and the method returns immediately. Defaults to `True`.

### `is_near_waypoint(self, actual: list, target: list, threshold: float = 2., gps=False)`

Determines if the `actual` position is within a `threshold` distance of the `target` position.

*   **`actual`** (`list`): The current position of the drone. Format depends on `gps` parameter:
    *   If `gps` is `True`: `[latitude, longitude]`
    *   If `gps` is `False`: `[North, East, Down]` (local NED coordinates).
*   **`target`** (`list`): The target position to compare against, matching the format of `actual`.
*   **`threshold`** (`float`, optional): The distance (in meters for local coordinates) within which the drone is considered "near" the target. For GPS checks (`gps=True`), this parameter is **ignored**; the method uses `self.lat_thresh` and `self.lon_thresh` derived from the `gps_thresh` provided in `__init__`. Defaults to `2` meters.
*   **`gps`** (`bool`, optional): If `True`, performs a GPS-based proximity check. If `False`, performs a Euclidean distance check on local coordinates. Defaults to `False`.

### `get_local_pos(self, frequency_hz=60)`

Retrieves the drone's current local position in the North-East-Down (NED) coordinate system, relative to the home position.

*   **`frequency_hz`** (`int`, optional): The desired frequency (in Hz) to request `LOCAL_POSITION_NED` messages from the drone. Defaults to `60`.
*   **Returns:** `list`: The local position as \( [\text{North (m), East (m), Down (m)}] \). Note that `Down` is positive, so an altitude of 10 meters above home would be represented by a `Z` value of -10 meters.

### `get_global_pos(self, time_tag=False, heading=False)`

Retrieves the drone's current global GPS position (Latitude, Longitude, Altitude relative to home).

*   **`time_tag`** (`bool`, optional): If `True`, includes the timestamp (time since boot in seconds) in the returned tuple. Defaults to `False`.
*   **`heading`** (`bool`, optional): If `True`, includes the drone's heading (degrees from North) in the returned tuple. Defaults to `False`.
*   **Returns:** `tuple`: Contains latitude (deg), longitude (deg), altitude (m), and optionally timestamp (s) and heading (deg) based on `time_tag` and `heading` parameters.

### `get_rc_value(self, channel)`

Retrieves the raw value of a specified RC (Radio Control) channel.

*   **`channel`** (`int`): The RC channel number to read (1-18).
*   **Returns:** `int` or `None`: The raw RC channel value (typically between 1000 and 2000), or `None` if the channel data is unavailable.

### `message_request(self, message_type, freq_hz=10)`

Sends a MAVLink command to configure the transmission interval for a specific message type, ensuring efficient data reception. This method prevents redundant interval requests.

*   **`message_type`** (`int`): The MAVLink message ID (e.g., `mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT`).
*   **`freq_hz`** (`int`, optional): The desired frequency (in Hz) for receiving the specified message. Defaults to `10`.

### `set_mode(self, mode: str)`

Sets the drone's flight mode.

*   **`mode`** (`str`): The string name of the desired flight mode (e.g., `'GUIDED'`, `'AUTO'`, `'RTL'`).

### `arm(self)`

Arms the drone's motors. The method will block execution until motor arming is confirmed by the drone.

### `takeoff(self, altitude=10, while_moving=None)`

Commands the drone to perform a vertical takeoff to a specified relative altitude. Requires the drone to be in `'GUIDED'` mode and armed.

*   **`altitude`** (`int`, optional): The target altitude in meters above the home position. Defaults to `10`.
*   **`while_moving`** (`callable`, optional): A function to execute repeatedly while the drone is ascending. Defaults to `None`.

### `connect_arm_takeoff(self, ip='tcp:127.0.0.1:5762', height=20)`

A convenient wrapper method to streamline the process of connecting, setting to GUIDED mode, arming, and initiating takeoff.

*   **`ip`** (`str`, optional): The IP address for MAVLink connection. Defaults to `'tcp:127.0.0.1:5762'`.
*   **`height`** (`int`, optional): The target takeoff altitude in meters. Defaults to `20`.

### `convert_to_global(self, local_delta: tuple, reference_point=None)`

Converts relative local NED coordinate displacements (North, East) from a reference point into global GPS coordinates.

*   **`local_delta`** (`tuple`): A tuple \( (\text{east\_delta\_meters, north\_delta\_meters}) \) representing the displacement.
*   **`reference_point`** (`list`, optional): The GPS coordinates \( [\text{latitude, longitude}] \) from which the `local_delta` is applied. If `None`, `self.home` (established during `__init__`) is used as the reference.
*   **Returns:** `list`: The calculated global GPS coordinates \( [\text{latitude, longitude}] \).

### `local_target(self, wp, acceptance_radius=5, while_moving=None, turn_into_wp=False)`

Sends a movement command to the drone using local North-East-Down (NED) coordinates relative to the home position.

*   **`wp`** (`list`): The target waypoint as \( [\text{North (m), East (m), Down (m)}] \). Remember that `Down` is positive, so a target altitude of 10 meters above home would be represented as a `Z` value of -10.
*   **`acceptance_radius`** (`int`, optional): The radial distance in meters within which the drone is considered to have reached the target. Defaults to `5`.
*   **`while_moving`** (`callable`, optional): A function to execute repeatedly while the drone is in transit. Defaults to `None`.
*   **`turn_into_wp`** (`bool`, optional): If `True`, the drone will adjust its yaw to align with the target waypoint as it approaches. Defaults to `False`.

### `speed_target(self, wp: list, yaw_rate=0)`

Sends a continuous velocity command to the drone in its body frame.

*   **`wp`** (`list`): A list specifying velocities as \( [\text{forward\_velocity (m/s), right\_velocity (m/s), down\_velocity (m/s)}] \). Note that `down_velocity` is positive for descending.
*   **`yaw_rate`** (`float`, optional): The desired angular velocity around the drone's vertical axis in degrees per second. Defaults to `0`.

### `RTL(self, while_moving=None)`

Commands the drone to perform a Return To Launch (RTL) maneuver. The method blocks until the drone has successfully landed and its motors are disarmed, then it closes the MAVLink connection.

*   **`while_moving`** (`callable`, optional): A function to execute repeatedly while the drone is performing the RTL. Defaults to `None`.

### `insert_coordinates_to_csv(self, file_path, coordinates, desc)`

Inserts a single set of GPS coordinates and an associated description into a CSV file. If the specified file does not exist, it will be created with a header row.

*   **`file_path`** (`str`): The full path to the CSV file (e.g., `'flight_log.csv'`).
*   **`coordinates`** (`tuple`): A tuple \( (\text{latitude, longitude}) \) of the coordinates to log.
*   **`desc`** (`str`): A string description to accompany the coordinates.

### `append_description_to_last_line(file_path, description)`

**Note:** This is a utility function defined within the class scope but does not operate on the `Zenmav` instance (`self`). It can be called directly as `Zenmav.append_description_to_last_line(file_path, description)`. It appends a description to the last data row of a CSV file.

*   **`file_path`** (`str`): The path to the CSV file.
*   **`description`** (`str`): The description string to append.

### `spiral_scan(self, largeur_detection=10, altitude=10, rayon_scan=100, safety_margin=0, center=None)`

Generates and executes a spiral flight path for scanning a circular area. The method calculates and prints the total time taken for the scan.

*   **`largeur_detection`** (`int`, optional): The effective horizontal detection width of the drone's payload in meters. This parameter determines the spacing between successive spiral passes. Defaults to `10`.
*   **`altitude`** (`int`, optional): The relative altitude in meters at which to perform the scan. Defaults to `10`.
*   **`rayon_scan`** (`int`, optional): The maximum radius of the circular area to be scanned, in meters. Defaults to `100`.
*   **`safety_margin`** (`int`, optional): An additional distance in meters added to `rayon_scan` to account for potential initial positioning errors or to ensure full coverage beyond the nominal radius. Defaults to `0`.
*   **`center`** (`list`, optional): Local NED coordinates \( [\text{N, E, D}] \) defining the center of the spiral scan. If `None`, the drone's current local position at the time the function is called will be used as the center.

### `rectilinear_scan(self, largeur_detection=10, altitude=10, rayon_scan=100, safety_margin=0, center=None)`

Generates and executes a rectilinear (lawnmower) flight path to scan a circular area efficiently. The method calculates and prints the total time taken for the scan.

*   **`largeur_detection`** (`int`, optional): The effective horizontal detection width of the drone's payload in meters. This dictates the spacing between parallel scan lines. Defaults to `10`.
*   **`altitude`** (`int`, optional): The relative altitude in meters at which to perform the scan. Defaults to `10`.
*   **`rayon_scan`** (`int`, optional): The maximum radius of the circular area to be scanned, in meters. Defaults to `100`.
*   **`safety_margin`** (`int`, optional): An additional distance in meters added to `rayon_scan` to compensate for initial positioning inaccuracies or to extend coverage. Defaults to `0`.
*   **`center`** (`list`, optional): Local NED coordinates \( [\text{N, E, D}] \) defining the center of the rectilinear scan. If `None`, the drone's current local position when the function is called will be used as the center.

### `generate_scan_points(scan_width=2, radius_of_scan=13)`

**Note:** This is an internal helper function intended to calculate the `x` and `y` coordinates for a rectilinear scan pattern. It is provided within the class but does not utilize instance-specific data. Be aware that in the provided source code, the `return x, y` statement is inside the loop, which may lead to incomplete point generation. It should be used for testing point generation logic rather than direct flight control.

*   **`scan_width`** (`int`, optional): The width of each scan pass. Defaults to `2`.
*   **`radius_of_scan`** (`int`, optional): The radius of the circular area for which to generate scan points. Defaults to `13`.
*   **Returns:** `tuple`: A tuple `(x, y)` where `x` and `y` are lists of local coordinates representing the path.

## Contributing

We welcome contributions from the community! If you have suggestions for improvements, encounter bugs, or wish to contribute code, please feel free to open an issue or pull request on our repository.

## License

This project is licensed under the Apache 2.0 License. See the `LICENSE` file for full details.
