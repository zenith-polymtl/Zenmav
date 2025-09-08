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
        self.mode_mapping = {'retract' : 0,
                              'neutral' : 1,
                                'mavlink_targeting' : 2,
                                  'rc_targeting':3,
                                  'gps_point' : 4,
                                  'sys_id_target' : 5,
                                  'home_location' : 6}
    
    def _validate_connection(self):
        """Validate that the connection supports gimbal commands"""
        # Check if MAVLink connection is active
        
        if not hasattr(self.connection, 'mav'):
            raise GimbalCommunicationError("Invalid MAVLink connection")
    
    def set_mode(self, mode : str):
        mode = mode.lower()
        """Configure gimbal mode using MAV_CMD_DO_MOUNT_CONTROL
        
        Args:
            mode: Gimbal mode (MODE_RETRACT, MODE_NEUTRAL, etc.)
            
        Returns:
            bool: True if command was sent successfully
            
        Raises:
            GimbalParameterError: If mode is invalid
            GimbalCommunicationError: If communication fails
        """
        if mode not in self.mode_mapping.keys():
            print(f"Invalid mode: {mode}")
            return
        
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
                self.mode_mapping[mode]  # param7 (mode)
            )
            self.current_mode = mode
            print(f'Gimbal mode set to {mode}')
            return True
        except Exception as e:
            raise GimbalCommunicationError(f"Failed to configure mode: {str(e)}")
    
    def retract(self):
        """Retract the gimbal
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.set_mode('retract')
    
    def neutral(self):
        """Set gimbal to neutral position
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.set_mode('neutral')
    
    def mavlink_targeting(self):
        """Switch to MAVLink targeting mode
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.set_mode('mavlink_targeting')
    
    def set_angle(self, pitch=None, yaw=None, pitch_rate=None, 
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
            print("Pitch must be between -180 and 180 degrees")
            return
        
        if yaw is not None and (yaw < -180 or yaw > 180):
            while yaw > 180 :
                yaw -= 360
            while yaw < -180:
                yaw += 360
        
        try:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                0,  # confirmation
                pitch if pitch is not None else self.current_angles['pitch'],
                yaw if yaw is not None else float('nan'),
                pitch_rate if pitch_rate is not None else self.current_angles['roll'],
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

            print(f'Moving gimbal to Pitch :  {pitch}, Yaw : {yaw}')

            return True
        except Exception as e:
            raise GimbalCommunicationError(f"Failed to set angle: {str(e)}")
    
    def point_down(self):
        """Point gimbal straight down (90-degree pitch)
        
        Returns:
            bool: True if command was sent successfully
        """
        return self.set_angle(pitch=-90, yaw=0)
    
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
            print("Latitude must be between -90 and 90 degrees")
            return
        
        if lon < -180 or lon > 180:
            print("Longitude must be between -180 and 180 degrees")
            return
        
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