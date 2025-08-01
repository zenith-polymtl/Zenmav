"""
Gimbal Control Module for Zenmav

This module provides comprehensive gimbal control functionality for ArduPilot-based drones
using MAVLink protocol. It extends the existing Zenmav core with robust gimbal control capabilities.

Key Features:
- MAVLink protocol integration for gimbal control
- Support for multiple gimbal modes
- Angle control with precision targeting
- ROI (Region of Interest) pointing capabilities
- Parameter management for gimbal configuration
- Comprehensive error handling and validation
- Integration with existing Zenmav systems

Dependencies:
- pymavlink
- zenmav.core
"""

from pymavlink import mavutil
import time

class GimbalError(Exception):
    """Base exception for gimbal operations"""
    pass

class GimbalCommunicationError(GimbalError):
    """Error in communication with gimbal"""
    pass

class GimbalParameterError(GimbalError):
    """Error with gimbal parameters"""
    pass

class GimbalTimeoutError(GimbalError):
    """Timeout waiting for gimbal response"""
    pass

class GimbalNotSupportedError(GimbalError):
    """Gimbal operation not supported"""
    pass

class GimbalController:
    """Main class for gimbal control operations"""
    
    # Gimbal modes
    MODE_RETRACT = 0
    MODE_NEUTRAL = 1
    MODE_MAVLINK_TARGETING = 2
    MODE_RC_TARGETING = 3
    MODE_GPS_POINT = 4
    MODE_SYSID_TARGET = 5
    MODE_HOME_LOCATION = 6
    
    # Yaw control flags
    YAW_FOLLOW = 0      # Body-frame/follow
    YAW_LOCK = 16       # Earth-frame/lock
    
    def __init__(self, drone):
        """Initialize gimbal controller with Zenmav drone instance
        
        Args:
            drone: Zenmav instance for communication
        """
        self.drone = drone
        self.connection = drone.connection
        self.current_mode = None
        self.current_angles = {"pitch": 0, "yaw": 0, "roll": 0}
        self._validate_connection()
    
    def _validate_connection(self):
        """Validate that the connection supports gimbal commands"""
        # Check if MAVLink connection is active
        if not hasattr(self.connection, 'mav'):
            raise GimbalCommunicationError("Invalid MAVLink connection")
    
    def configure_mode(self, mode):
        """Configure gimbal mode using MAV_CMD_DO_MOUNT_CONTROL
        
        Args:
            mode: Gimbal mode (MODE_RETRACT, MODE_NEUTRAL, etc.)
            
        Returns:
            bool: True if command was sent successfully
            
        Raises:
            GimbalParameterError: If mode is invalid
            GimbalCommunicationError: If communication fails
        """
        if mode not in [self.MODE_RETRACT, self.MODE_NEUTRAL, self.MODE_MAVLINK_TARGETING,
                       self.MODE_RC_TARGETING, self.MODE_GPS_POINT, self.MODE_SYSID_TARGET,
                       self.MODE_HOME_LOCATION]:
            raise GimbalParameterError(f"Invalid mode: {mode}")
        
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
                0,  # confirmation
                0,  # param1 (pitch)
                0,  # param2 (roll)
                0,  # param3 (yaw)
                0,  # param4 (altitude)
                0,  # param5 (longitude)
                0,  # param6 (latitude)
                mode  # param7 (mode)
            )
            self.current_mode = mode
            return True
        except Exception as e:
            raise GimbalCommunicationError(f"Failed to configure mode: {str(e)}")
    
    def retract(self):
        """Retract the gimbal
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.configure_mode(self.MODE_RETRACT)
    
    def neutral(self):
        """Set gimbal to neutral position
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.configure_mode(self.MODE_NEUTRAL)
    
    def mavlink_targeting(self):
        """Switch to MAVLink targeting mode
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.configure_mode(self.MODE_MAVLINK_TARGETING)
    
    def set_angle(self, pitch=None, yaw=None, roll=None, pitch_rate=None, 
                  yaw_rate=None, flags=YAW_FOLLOW):
        """Set gimbal angles using MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        
        Args:
            pitch: Pitch angle in degrees (positive is up)
            yaw: Yaw angle in degrees (positive is clockwise)
            roll: Roll angle in degrees
            pitch_rate: Pitch rate in deg/s (positive is up)
            yaw_rate: Yaw rate in deg/s (positive is clockwise)
            flags: YAW_FOLLOW or YAW_LOCK for yaw behavior
            
        Returns:
            bool: True if command was sent successfully
            
        Raises:
            GimbalParameterError: If parameters are invalid
            GimbalCommunicationError: If communication fails
        """
        # Validate parameters
        if pitch is not None and (pitch < -180 or pitch > 180):
            raise GimbalParameterError("Pitch must be between -180 and 180 degrees")
        
        if yaw is not None and (yaw < -180 or yaw > 180):
            raise GimbalParameterError("Yaw must be between -180 and 180 degrees")
        
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,  # confirmation
                pitch if pitch is not None else float('nan'),
                yaw if yaw is not None else float('nan'),
                pitch_rate if pitch_rate is not None else float('nan'),
                yaw_rate if yaw_rate is not None else float('nan'),
                flags,  # param5 (flags)
                0,      # param6 (unused)
                0       # param7 (gimbal device ID)
            )
            
            # Update current angles if specified
            if pitch is not None:
                self.current_angles["pitch"] = pitch
            if yaw is not None:
                self.current_angles["yaw"] = yaw
            if roll is not None:
                self.current_angles["roll"] = roll
                
            return True
        except Exception as e:
            raise GimbalCommunicationError(f"Failed to set angle: {str(e)}")
    
    def point_down(self):
        """Point gimbal straight down (90-degree pitch)
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.set_angle(pitch=90, yaw=0)
    
    def point_forward(self):
        """Point gimbal forward (0-degree pitch, 0-degree yaw)
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.set_angle(pitch=0, yaw=0)
    
    def point_at_location(self, lat, lon, alt, frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT):
        """Point gimbal at specific GPS location using MAV_CMD_DO_SET_ROI_LOCATION
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
            frame: MAVLink frame type
            
        Returns:
            bool: True if command was sent successfully
            
        Raises:
            GimbalParameterError: If coordinates are invalid
            GimbalCommunicationError: If communication fails
        """
        # Validate parameters
        if lat < -90 or lat > 90:
            raise GimbalParameterError("Latitude must be between -90 and 90 degrees")
        
        if lon < -180 or lon > 180:
            raise GimbalParameterError("Longitude must be between -180 and 180 degrees")
        
        try:
            self.connection.mav.command_int_send(
                self.connection.target_system,
                self.connection.target_component,
                frame,
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
                0,  # current
                0,  # autocontinue
                0,  # param1 (gimbal device id)
                0,  # param2
                0,  # param3
                0,  # param4
                int(lat * 1e7),  # param5 (latitude in degrees * 10^7)
                int(lon * 1e7),  # param6 (longitude in degrees * 10^7)
                float(alt)       # param7 (altitude in meters)
            )
            return True
        except Exception as e:
            raise GimbalCommunicationError(f"Failed to point at location: {str(e)}")
    
    def stop_pointing(self):
        """Stop pointing at ROI using MAV_CMD_DO_SET_ROI_NONE
        
        Returns:
            bool: True if command was sent successfully
        """
        try:
            self.connection.mav.command_int_send(
                self.connection.target_system,
                self.connection.target_component,
                0,  # frame
                mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE,
                0,  # current
                0,  # autocontinue
                0,  # param1
                0,  # param2
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0   # param7
            )
            return True
        except Exception as e:
            raise GimbalCommunicationError(f"Failed to stop pointing: {str(e)}")
    
    def get_gimbal_param(self, param_name):
        """Get gimbal-specific parameter
        
        Args:
            param_name: Name of the parameter (without MNT1_ prefix)
            
        Returns:
            Parameter value
            
        Raises:
            GimbalParameterError: If parameter name is invalid
        """
        if not param_name or not isinstance(param_name, str):
            raise GimbalParameterError("Parameter name must be a non-empty string")
        
        full_param_name = f"MNT1_{param_name}"
        return self.drone.get_param(full_param_name)
    
    def set_gimbal_param(self, param_name, value):
        """Set gimbal-specific parameter
        
        Args:
            param_name: Name of the parameter (without MNT1_ prefix)
            value: Value to set
            
        Returns:
            bool: True if parameter was set successfully
            
        Raises:
            GimbalParameterError: If parameter name or value is invalid
        """
        if not param_name or not isinstance(param_name, str):
            raise GimbalParameterError("Parameter name must be a non-empty string")
        
        full_param_name = f"MNT1_{param_name}"
        return self.drone.set_param(full_param_name, value)
    
    def get_status(self):
        """Get current gimbal status
        
        Returns:
            dict: Dictionary with status information including:
                - mode: Current gimbal mode
                - angles: Current pitch, yaw, roll angles
                - health: Gimbal health status
        """
        status = {
            "mode": self.current_mode,
            "angles": self.current_angles.copy(),
            "health": "unknown"
        }
        
        # Try to get actual gimbal status from messages
        try:
            # Request GIMBAL_DEVICE_ATTITUDE_STATUS message
            self.drone.message_request(mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, 1)
            
            # Try to receive the message
            msg = self.connection.recv_match(type='GIMBAL_DEVICE_ATTITUDE_STATUS', blocking=True, timeout=1)
            if msg:
                status["health"] = "ok"
                # Extract attitude information if available
        except:
            pass
            
        return status