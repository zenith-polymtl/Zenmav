# Zenmav Gimbal Control Documentation

This document provides comprehensive documentation for the gimbal control functionality in Zenmav.

## Overview

The gimbal control module extends Zenmav with robust gimbal functionality using ArduPilot MAVLink protocol. It provides precise control over camera gimbals for various applications including surveillance, mapping, and object tracking.

## Installation

The gimbal control functionality is included in Zenmav version 0.0.6 and later. No additional installation is required.

## Quick Start

```python
from zenmav.core import Zenmav

# Initialize drone
drone = Zenmav()

# Point gimbal down
drone.gimbal_point_down()

# Set custom angles
drone.gimbal_set_angle(pitch=45, yaw=90)

# Point at specific location
drone.gimbal_point_at_location(lat=37.7749, lon=-122.4194, alt=100)
```

## Gimbal Control Methods

### Direct Gimbal Control

#### `gimbal_set_angle(pitch=None, yaw=None, roll=None)`

Set gimbal angles using MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW.

**Parameters:**
- `pitch` (float, optional): Pitch angle in degrees (positive is up)
- `yaw` (float, optional): Yaw angle in degrees (positive is clockwise)
- `roll` (float, optional): Roll angle in degrees

**Returns:**
- `bool`: True if command was sent successfully

**Example:**
```python
# Set pitch to 30 degrees, yaw to 45 degrees
drone.gimbal_set_angle(pitch=30, yaw=45)
```

#### `gimbal_point_down()`

Point gimbal straight down (90-degree pitch).

**Returns:**
- `bool`: True if command was sent successfully

**Example:**
```python
# Point gimbal straight down
drone.gimbal_point_down()
```

#### `gimbal_point_forward()`

Point gimbal forward (0-degree pitch, 0-degree yaw).

**Returns:**
- `bool`: True if command was sent successfully

**Example:**
```python
# Point gimbal forward
drone.gimbal_point_forward()
```

### Mode Control

#### `gimbal_retract()`

Retract the gimbal using MAV_CMD_DO_MOUNT_CONTROL.

**Returns:**
- `bool`: True if command was sent successfully

**Example:**
```python
# Retract gimbal
drone.gimbal_retract()
```

#### `gimbal_neutral()`

Set gimbal to neutral position using MAV_CMD_DO_MOUNT_CONTROL.

**Returns:**
- `bool`: True if command was sent successfully

**Example:**
```python
# Set gimbal to neutral position
drone.gimbal_neutral()
```

#### `gimbal_mavlink_targeting()`

Switch to MAVLink targeting mode using MAV_CMD_DO_MOUNT_CONTROL.

**Returns:**
- `bool`: True if command was sent successfully

**Example:**
```python
# Switch to MAVLink targeting mode
drone.gimbal_mavlink_targeting()
```

### Region of Interest (ROI) Control

#### `gimbal_point_at_location(lat, lon, alt)`

Point gimbal at specific GPS location using MAV_CMD_DO_SET_ROI_LOCATION.

**Parameters:**
- `lat` (float): Latitude in degrees
- `lon` (float): Longitude in degrees
- `alt` (float): Altitude in meters

**Returns:**
- `bool`: True if command was sent successfully

**Example:**
```python
# Point at specific coordinates
drone.gimbal_point_at_location(lat=37.7749, lon=-122.4194, alt=100)
```

## Advanced Gimbal Control

For more advanced control, you can access the underlying `GimbalController` object directly:

```python
# Access the gimbal controller directly
gimbal_controller = drone.gimbal

# Use advanced methods
gimbal_controller.set_angle(pitch=45, yaw=90, flags=GimbalController.YAW_LOCK)
```

### GimbalController Methods

#### `set_angle(pitch=None, yaw=None, roll=None, pitch_rate=None, yaw_rate=None, flags=0)`

Set gimbal angles with additional options.

**Parameters:**
- `pitch` (float, optional): Pitch angle in degrees
- `yaw` (float, optional): Yaw angle in degrees
- `roll` (float, optional): Roll angle in degrees
- `pitch_rate` (float, optional): Pitch rate in deg/s
- `yaw_rate` (float, optional): Yaw rate in deg/s
- `flags` (int): YAW_FOLLOW (0) or YAW_LOCK (16) for yaw behavior

#### `point_at_location(lat, lon, alt, frame=6)`

Point gimbal at specific GPS location with frame specification.

#### `get_status()`

Get current gimbal status including mode and angles.

#### `get_gimbal_param(param_name)`

Get gimbal-specific parameter.

#### `set_gimbal_param(param_name, value)`

Set gimbal-specific parameter.

## Error Handling

The gimbal control module provides comprehensive error handling:

- `GimbalError`: Base exception for gimbal operations
- `GimbalCommunicationError`: Error in communication with gimbal
- `GimbalParameterError`: Error with gimbal parameters
- `GimbalTimeoutError`: Timeout waiting for gimbal response
- `GimbalNotSupportedError`: Gimbal operation not supported

**Example:**
```python
try:
    drone.gimbal_point_down()
except GimbalParameterError as e:
    print(f"Gimbal parameter error: {e}")
except GimbalCommunicationError as e:
    print(f"Gimbal communication error: {e}")
```

## Configuration Parameters

Common gimbal parameters that can be accessed using `get_gimbal_param()` and `set_gimbal_param()`:

- `PITCH_MIN`: Minimum pitch angle
- `PITCH_MAX`: Maximum pitch angle
- `ROLL_MIN`: Minimum roll angle
- `ROLL_MAX`: Maximum roll angle
- `YAW_MIN`: Minimum yaw angle
- `YAW_MAX`: Maximum yaw angle
- `RC_RATE`: RC control rate

**Example:**
```python
# Get minimum pitch angle
min_pitch = drone.gimbal.get_gimbal_param('PITCH_MIN')

# Set maximum yaw angle
drone.gimbal.set_gimbal_param('YAW_MAX', 180)
```

## Integration with Flight Operations

The gimbal control module integrates seamlessly with existing Zenmav flight operations:

```python
from zenmav.core import Zenmav

def complete_mission_with_gimbal():
    # Initialize drone
    drone = Zenmav()
    
    # Standard flight operations
    drone.arm()
    drone.set_mode("GUIDED")
    drone.takeoff(altitude=20)
    
    # Gimbal operations during flight
    drone.gimbal_point_down()  # Point camera down for takeoff inspection
    
    # Navigate to waypoint
    drone.local_target([30, 20, 0])
    
    # Point gimbal at specific location for surveillance
    drone.gimbal_point_at_location(lat=37.7749, lon=-122.4194, alt=100)
    
    # Return to launch
    drone.RTL()
```

## Best Practices

1. **Always check gimbal status** before critical operations
2. **Use appropriate error handling** for robust applications
3. **Set reasonable angle limits** to prevent mechanical damage
4. **Test in simulation** before real-world deployment
5. **Monitor gimbal health** during extended operations

## Troubleshooting

### Common Issues

1. **Gimbal not responding**: Check physical connections and power supply
2. **Incorrect angles**: Verify parameter limits and coordinate system
3. **Communication errors**: Ensure proper MAVLink configuration

### Debugging Tips

```python
# Check gimbal status
status = drone.gimbal.get_status()
print(f"Gimbal status: {status}")

# Check specific parameters
pitch_min = drone.gimbal.get_gimbal_param('PITCH_MIN')
print(f"Minimum pitch: {pitch_min}")
```

## Example Applications

### Surveillance Mission

```python
def surveillance_mission():
    drone = Zenmav()
    drone.arm()
    drone.set_mode("GUIDED")
    drone.takeoff(altitude=30)
    
    # Point camera down for initial inspection
    drone.gimbal_point_down()
    time.sleep(5)
    
    # Point at surveillance target
    drone.gimbal_point_at_location(lat=37.7749, lon=-122.4194, alt=0)
    
    # Hold position and monitor
    time.sleep(30)
    
    drone.RTL()
```

### Mapping Mission

```python
def mapping_mission():
    drone = Zenmav()
    drone.arm()
    drone.set_mode("GUIDED")
    drone.takeoff(altitude=50)
    
    # Keep camera pointed down for mapping
    drone.gimbal_point_down()
    
    # Perform survey pattern
    drone.spiral_scan(detection_width=8, altitude=50, scan_radius=100)
    
    drone.RTL()
```

## API Reference

For detailed API documentation, see the main Zenmav documentation which includes all gimbal control methods in the Gimbal Control Utilities section.