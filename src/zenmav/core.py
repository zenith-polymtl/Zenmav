from .zenrelay import MavRelay, SYSID_IN_USE_PREFIX, is_vehicle_heartbeat, mavutil
from .zenlink import SharedReader, lock_sends, default_system_id, free_system_id
from pymavlink import mavparm
from pymavlink.mavftp import MAVFTP
import os
import time
import numpy as np
import csv
from math import atan2
from geopy.distance import distance
from geopy import Point
from datetime import datetime
import threading
from .zenboundary import Limits
from .zenpoint import wp
from .zengimbal import GimbalController
from .zenparams import LEGACY, SI, PROBE_PARAM, naming_for_version, translate


class Zenmav:
    def __init__(
        self,
        ip: str = "tcp:127.0.0.1:5762",
        baud=None,
        gps_thresh: float = None,
        GCS=False,
        tcp_ports=[14551],
        boundary_path=None,
    ):
        """Initializes the Zenmav class, allowing connection to a drone via MAVLink protocol."""
        self.gps_thresh = gps_thresh  # GPS threshold in meters
        self.relay = None
        self._requested_rates = {}
        self._heartbeat_stop = threading.Event()
        self._link_closed = False
        if GCS:
            self.relay = MavRelay(self._open_link(ip, baud), tcp_ports=[14550, *tcp_ports])

        self.connect(ip, baud)
        self.ap_version = self.get_autopilot_version()
        self.param_naming = self.detect_param_naming()
        nav_thresh = self.get_param("WP_RADIUS_M")
        
        if self.gps_thresh is None:
            self.gps_thresh = nav_thresh + 1.0
        else:
            if nav_thresh > self.gps_thresh:
                while True:
                    print(
                        f"WARNING : Zenmav threshold {self.gps_thresh} is less than the AP nav threshold {nav_thresh}."
                    )
        self.set_home()
        try:
            self.home.show()
        except:
            self.home = self.get_global_pos()
            print('Measured initial global pose to set home')

        ref_point = Point(self.home.lat, self.home.lon)
        point_north = distance(meters=self.gps_thresh).destination(ref_point, bearing=0)
        self.lat_thresh = abs(point_north.latitude - ref_point.latitude)

        point_east = distance(meters=self.gps_thresh).destination(ref_point, bearing=90)
        self.lon_thresh = abs(point_east.longitude - ref_point.longitude)

        if boundary_path is not None:
            self.limits = Limits(self, boundary_path, check_interval=0.25)
        
        self.gimbal = GimbalController(self)
        self.parms = mavparm.MAVParmDict() 
        print("Zenmav initialized")

    def _open_link(self, ip: str, baud=None, source_system=255):
        """Opens a pymavlink connection. TCP and serial links reconnect by themselves."""
        kwargs = {"source_system": source_system, "autoreconnect": True}
        if baud is not None:
            kwargs["baud"] = baud
        return mavutil.mavlink_connection(ip, **kwargs)

    def connect(self, ip_address: str = "tcp:127.0.0.1:5762", baud: int = None):
        """Enables easy connection to the drone, and waits for heartbeat to ensure a live communication. Only call this function once it init, should NOT be run outside of init.

        Only the heartbeat of an autopilot is accepted as the vehicle: heartbeats of ground stations
        (Mission Planner, other Zenmav instances), gimbals or companion computers are ignored.
        The MAVLink system ID of Zenmav is chosen automatically: the port number modulo 255, or a free
        ID between 200 and 254 when another system on the link already uses it.

        Args:
            ip_address (str, optional): IP address for connection.
                Sitl simulation : 'tcp:127.0.0.1:5762' .
                Real connection : 'udp:<ip_ubuntu>:14551' (Ensure the antenna signal is properly transmitted on this port and UDP communication is allocated between windows-ubuntu).
                Zenith Siyi connection : 'udpout:192.168.144.12:19856'
            baud (int, optional): Baud rate of serial links.
        Returns:
            None
        """
        if self.relay is not None:
            preferred = 14550 % 255  # System ID of previous versions in GCS mode
        else:
            preferred = default_system_id(ip_address)
        provisional = free_system_id(set(), preferred=preferred, salt=os.getpid())

        if self.relay is not None:
            self.connection = self.relay.add_internal_link(provisional)
        else:
            self.connection = self._open_link(ip_address, baud, provisional)
        connection = self.connection
        lock_sends(connection)

        vehicle, others = self._wait_for_vehicle()
        connection.target_system, connection.target_component = vehicle
        system_id = free_system_id(others | {vehicle[0]}, preferred=provisional, salt=os.getpid())
        if system_id != provisional:
            print(f"System ID {provisional} is already used on this link")
        connection.mav.srcSystem = connection.source_system = system_id
        print(f"System ID : {system_id}")

        self._reader = SharedReader(connection)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, name="zenmav-heartbeat", daemon=True)
        self._heartbeat_thread.start()

    def _wait_for_vehicle(self, warning_period: float = 5.0, observation_window: float = 1.5):
        """Waits for an autopilot heartbeat, while noting the system IDs already present on the link.

        Returns:
            tuple: ((sysid, compid) of the vehicle, set of the other system IDs seen)
        """
        connection = self.connection
        print("Waiting for heartbeat...")
        start = last_heartbeat = last_warning = time.monotonic()
        self._send_heartbeat()
        vehicle = None
        others = set()
        while True:
            msg = connection.recv_match(blocking=True, timeout=0.2)
            now = time.monotonic()
            if now - last_heartbeat >= 1.0:
                self._send_heartbeat()
                last_heartbeat = now
            if msg is not None and msg.get_type() != "BAD_DATA":
                text = str(getattr(msg, "text", ""))
                if vehicle is None and is_vehicle_heartbeat(msg):
                    vehicle = (msg.get_srcSystem(), msg.get_srcComponent())
                    print("Heartbeat received!")
                elif msg.get_type() == "STATUSTEXT" and text.startswith(SYSID_IN_USE_PREFIX):
                    others.add(int(text[len(SYSID_IN_USE_PREFIX):]))  # Sent by a Zenmav relay
                elif msg.get_srcSystem() != 0:
                    others.add(msg.get_srcSystem())
            if vehicle is not None and now - start >= observation_window:
                others.discard(vehicle[0])
                return vehicle, others
            if vehicle is None and now - last_warning >= warning_period:
                last_warning = now
                seen = f" Other MAVLink systems seen: {sorted(others)}." if others else ""
                print(f"Still waiting for a vehicle heartbeat...{seen}")

    def _send_heartbeat(self):
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE
        )

    def _heartbeat_loop(self):
        """Sends a GCS heartbeat every second, so other MAVLink systems know this Zenmav instance."""
        while not self._heartbeat_stop.wait(1.0):
            try:
                self._send_heartbeat()
            except Exception:
                pass

    def _wait_vehicle_heartbeat(self, timeout: float = 1.0):
        """Returns the next heartbeat of the vehicle, or None after timeout seconds."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            msg = self.connection.recv_match(type="HEARTBEAT", blocking=True, timeout=remaining)
            if msg is not None and msg.get_srcSystem() == self.connection.target_system and is_vehicle_heartbeat(msg):
                return msg

    def global_target(self, waypoint: wp, while_moving=None, wait_to_reach: bool = True, heading = None):
        """Sends a movement command to the drone for a specific global GPS coordinate.

        Args:
            wp (tuple): Target waypoint as (latitude, longitude, altitude in meters).
            acceptance_radius (float, optional): Distance at which the target is considered reached. Defaults to 5 meters. Deprecated, use gps_thresh in Zenmav init.
            while_moving (function, optional) to execute while the drone is in transit.
            wait_to_reach (bool, optional): Whether to wait for the drone to reach the target before proceeding.
        """

        if isinstance(waypoint, (list, tuple)):
            waypoint = wp(waypoint[0],waypoint[1],waypoint[2], frame = 'global')

        connection = self.connection
        #print(waypoint.coordinates)

        if waypoint.frame == "local":
            print('local frame, converting to global')
            waypoint = self.convert_to_global(waypoint, self.home)

        if heading == None:
            mask = 0b11011111000
            heading = 0
        else:
            mask =  0b1001111000
            heading *= 3.1415926535/180

        
        # Send a MAVLink command to set the target global position
        connection.mav.set_position_target_global_int_send(
            0,  # Timestamp in milliseconds
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Global frame with relative altitude
            mask,  # Position mask
            int(waypoint.lat * 1e7),  # Latitude in degrees * 1e7
            int(waypoint.lon * 1e7),  # Longitude in degrees * 1e7
            waypoint.alt,  # Altitude in meters (relative to home)
            0,
            0,
            0,  # No velocity set
            0,
            0,
            0,  # No acceleration set
            heading,
            0,  # No yaw or yaw rate
        )

        if wait_to_reach:
            # Wait for the waypoint to be reached
            print("Waiting for waypoint to be reached...")
            while not self.is_near_waypoint(self.get_global_pos(), waypoint):
                if while_moving is not None:
                    while_moving()
                else:
                    pass
            else:
                print("Waypoint reached!")
    
    def convert_to_global(self, local_pos: list, reference_point: list = None):
        if reference_point is None:
            reference_point = self.home.lat, self.home.lon
        """Converts local NED coordinates to global GPS coordinates.

        Args:
            local_pos (list): Local position in NED system [N, E].

        Returns:
            tuple: Global GPS position (latitude, longitude, altitude).
        """
        if isinstance(reference_point, wp):
            reference_point = reference_point.lat, reference_point.lon

        point_north = distance(meters=local_pos.N).destination(reference_point, bearing=0)
        final_point = distance(meters=local_pos.E).destination(point_north, bearing=90)
        local_pos.lat = final_point.latitude
        local_pos.lon = final_point.longitude
        local_pos.alt = -local_pos.D
        local_pos.frame = "global"
        new_pos = local_pos

        return new_pos
    
    def convert_to_local(self, global_pos, reference_point=None):
        """
        Convert global GPS coordinates to local NED meters relative to a reference.
        Inputs:
            global_pos: wp or (lat, lon, alt), where alt is relative to reference (e.g., home).
            reference_point: wp or (lat, lon, alt). Defaults to self.home.
        Returns:
            wp(N, E, D, frame="local")
        """
        # Normalize inputs
        if isinstance(global_pos, (list, tuple)):
            global_pos = wp(global_pos[0], global_pos[1], global_pos[2], frame="global")

        if reference_point is None:
            ref_lat, ref_lon = self.home.lat, self.home.lon
        else:
            if isinstance(reference_point, wp):
                ref_lat, ref_lon = reference_point.lat, reference_point.lon
            else:
                ref_lat, ref_lon = reference_point[0], reference_point[1]

        # Northing (meters): move in latitude with same longitude
        dN = distance((ref_lat, ref_lon), (global_pos.lat, ref_lon)).meters
        if global_pos.lat < ref_lat:
            dN = -dN

        # Easting (meters): move in longitude with same latitude
        dE = distance((ref_lat, ref_lon), (ref_lat, global_pos.lon)).meters
        if global_pos.lon < ref_lon:
            dE = -dE

        # Down (meters): NED convention (Down positive). If alt is relative to reference,
        # local D = - (alt - ref_alt). With relative_alt (already relative to home), this is just -alt.
        D = - global_pos.alt

        return wp(dN, dE, D, frame="local")


    def local_target(
        self,
        waypoint : wp,
        acceptance_radius: float = 5.0,
        while_moving=None,
        turn_into_wp: bool = False,
        wait_to_reach: bool = True,
        heading = None
    ):
        """Allows easy sending of a drone movement command to local coordinates in NED system.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            wp (list): list of coordinates in local coordinate system [N, E, D] (YES POSITIVE ALTITUDE = NEGATIVE)
            while_moving (function) : Thing to do while waiting for wp to be reached
            acceptance_radius (int, optional): Distance at which the drone considers the target reached. Defaults to 5.
        """
        if isinstance(waypoint, (list, tuple)):
            waypoint = wp(waypoint[0],waypoint[1],waypoint[2], frame = 'local')

        connection = self.connection

        if heading == None and turn_into_wp == False:
            mask = 0b11011111000
            heading = 0
        else:
            mask =  0b1001111000
            if not turn_into_wp:
                heading *= 3.1415926535/180



        if turn_into_wp:
            actual_pos = self.get_local_pos()
            actual_x, actual_y = actual_pos.N, actual_pos.E
            heading = atan2(waypoint.E - actual_y, waypoint.N - actual_x)

        connection.mav.set_position_target_local_ned_send(
            0,  # Time in milliseconds
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            mask,  # Position mask
            waypoint.N,
            waypoint.E,
            waypoint.D,  # X (North), Y (East), Z (Down)
            0,
            0,
            0,  # No velocity
            0,
            0,
            0,  # No acceleration
            heading,
            0,  # No yaw or yaw rate
        )
        if wait_to_reach:
            # Wait for the waypoint to be reached
            print("Waiting for waypoint to be reached...")
            while not self.is_near_waypoint(
                self.get_local_pos(), waypoint, threshold=acceptance_radius
            ):
                if while_moving is not None:
                    while_moving()
                else:
                    pass
            else:
                print("Waypoint reached!")

    def speed_target(self, waypoint: wp, yaw_rate = None):
          # Convert degrees to radians
        """Allows easy sending of a drone speed command in its reference system (Forward, right, down).

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            wp (list): list of coordinates in system of [Forward, Right, Down] (YES POSITIVE ALTITUDE = NEGATIVE)
            yaw_rate (float, optional): Speed of drone rotation around its vertical axis. Defaults to 0. IN degrees per second.
        """

        connection = self.connection
        if isinstance(waypoint, (list, tuple)):
            waypoint = wp(waypoint[0],waypoint[1],waypoint[2], frame = 'base_link')
        else:
            if waypoint.frame == "local" or waypoint.frame == "global":
                raise(ValueError)
        
        
        if yaw_rate == None:
            mask = 0b111111000111
            yaw_rate = 0
        else:
            mask =  0b010111000111
            yaw_rate = yaw_rate * np.pi / 180

        connection.mav.set_position_target_local_ned_send(
            0,  # Time in milliseconds
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            mask, # Speed mask
            0,
            0,
            0,  # X Front, Y Right, Z Down
            waypoint.F,
            waypoint.R,
            waypoint.D,  # No velocity
            0,
            0,
            0,  # No acceleration
            0,  # No yaw or
            yaw_rate,  # yaw rate
        )

        # Wait for the waypoint to be reached
        #print(f"Speed command of {waypoint.coordinates} m/s")

    def yaw_target(self,yaw_angle, max_rate =45,  relative:bool = False, clockwise = -1):

        # make sure the angle is between 0 and 360
        while yaw_angle<0:
            yaw_angle += 360
        while yaw_angle > 360:
            yaw_angle -= 360

        self.connection.mav.command_long_send(  
        self.connection.target_system,           # target_system  
        self.connection.target_component,        # target_component  
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command  
        0,                                  # confirmation  
        yaw_angle,                                 # param1: target angle (0-360 degrees, 0=north)  
        max_rate,                                 # param2: angular speed (deg/s)  
        0,                                  # param3: direction (1=clockwise, -1=counter-clockwise)  
        relative,                                  # param4: relative (0=absolute, 1=relative offset)  
        0, 0, 0                            # param5-7: empty  
        )


    def is_near_waypoint(
        self, actual: wp, target: wp, threshold: float = 2.0):
        """Returns True if the distance between the drone and the target is < threshold. Else False.

        Args:
            actual (list): Current drone position (Local NED coordinates : [N, E, -Z])
            target (list): Target position to compare (Local NED coordinates : [N, E, -Z])
            threshold (float, optional): Distance at which to return True. Defaults to 2.

        Returns:
            bool: True if drone is close enough, False otherwise
        """
        if not isinstance(actual, wp):
            return abs(actual - target) < threshold

        if actual.frame == "global":
            return (abs(actual.lat - target.lat) <= self.lat_thresh) and (
                abs(actual.lon - target.lon) <= self.lon_thresh
            )
        else:
            #print(f'ACTUAL : {actual.coordinates} / REAL : {target.coordinates}')
            error = np.linalg.norm(np.array(actual.coordinates) - np.array(target.coordinates)) 
            #print(error)
            return error < threshold

    def _next_message(self, msg_type: str, msg_id: int, freq_hz: float, warning_after: float = 2.0):
        """Requests msg_type at freq_hz, discards the copies already received and waits for the next one.

        Waits as long as needed, printing a warning and requesting the message again every
        warning_after seconds without data (link lost, vehicle rebooted...).
        """
        self.message_request(msg_id, freq_hz)
        connection = self.connection
        while connection.recv_match(type=msg_type, blocking=False):
            pass  # Discard old messages
        while True:
            msg = connection.recv_match(type=msg_type, blocking=True, timeout=warning_after)
            if msg is not None:
                return msg
            if self._reader.closed:
                raise ConnectionError("MAVLink connection is closed")
            print(f"WARNING : no {msg_type} received for {warning_after:.0f} s, requesting it again")
            self._requested_rates.pop(msg_id, None)
            self.message_request(msg_id, freq_hz)

    def get_local_pos(self, frequency_hz: int = 60):
        """Allows to get the local position, and makes a request to get the data at the desired frequency.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            frequency_hz (int, optional): Frequency of data requests. Defaults to 60.

        Returns:
            Position (wp): Position in local NED coordinate system : [N, E, -Z])
        """
        msg = self._next_message(
            "LOCAL_POSITION_NED", mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, frequency_hz
        )
        return wp(msg.x, msg.y, msg.z, frame="local")


    def get_global_pos(self, time_tag: bool = False, heading: bool = False):
        """Gets the current global position of the drone in GPS coordinates (latitude, longitude, altitude). Optionally includes a time tag and heading.

        Args:
            time_tag (bool, optional): Include the timestamp of the data. Defaults to False.
            heading (bool, optional): Include the heading as last value in the tuple return. Defaults to False.

        Returns:
            tuple : [timestamp, latitude, longitude, altitude, heading]
            timestamp and heading are optional, depending on the parameters.
        """
        msg = self._next_message(
            "GLOBAL_POSITION_INT", mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 60
        )
        lat = msg.lat / 1e7  # Convert from int32 to degrees
        lon = msg.lon / 1e7  # Convert from int32 to degrees
        alt = msg.relative_alt / 1000.0  # Convert from mm to meters (relative altitude)

        pos = wp(lat, lon, alt, frame = "global")
        if heading:
            pos.hdg = msg.hdg / 100
        if time_tag:
            pos.timestamp = msg.time_boot_ms / 1000.0  # Seconds since autopilot boot
        return pos

    def get_rc_value(self, channel: int):
        """
        Retrieve the raw value of an RC channel.

        Args:
            connection: MAVLink connection object.
            channel (int): Channel number to read (1-18).

        Returns:
            int: Raw RC channel value (1000 - 2000).
        """
        msg = self._next_message("RC_CHANNELS", mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 60)
        # Channel values are indexed from 1 to 18
        value = getattr(msg, f"chan{channel}_raw", None) if 1 <= channel <= 18 else None
        if value is None:
            print(f"Channel {channel} not available in the message.")
        return value
                    
    def get_param(self, param_name: str, max_retries=10, timeout=3):
        """Fetches a specific parameter from the drone.

        ArduPilot 4.6 (WPNAV_*) and 4.7 (WP_*) names are both accepted and translated to the
        connected firmware's naming. The value is returned in the units of the requested name
        (e.g. WPNAV_SPEED in cm/s, WP_SPD in m/s). See zenparams.py.
        """
        fw_name, factor = translate(param_name, self.param_naming)
        if fw_name != param_name:
            print(f"Parameter {param_name} is named {fw_name} on this firmware")
        value = self._fetch_param(fw_name, max_retries, timeout)
        if value is None:
            return None
        return value / factor

    def _fetch_param(self, param_name: str, max_retries=10, timeout=3):
        """Fetches a parameter by its exact firmware name. Returns None if it can't be read."""
        for i in range(max_retries):
            self._reader.flush()  # Old PARAM_VALUE copies are not the answer to this request
            self.connection.param_fetch_one(param_name)
            print(f"Requesting parameter: {param_name}")

            msg = self.connection.recv_match(type=["PARAM_VALUE", "PARAM_ERROR"], blocking=True, timeout=timeout)
            if msg and msg.param_id == param_name:
                if msg.get_type() == "PARAM_ERROR":  # Sent by recent firmwares when the parameter does not exist
                    print(f"Parameter {param_name} does not exist on this firmware")
                    return None
                param_value = msg.param_value
                print(f"Parameter {param_name}: {param_value}")
                if param_value is not None:
                    return param_value
        return None

    def set_param(self, param_name: str, value: float, max_retries=5, parm_type=None):
        """Sets a specific parameter on the drone using MAVParmDict.

        ArduPilot 4.6 (WPNAV_*) and 4.7 (WP_*) names are both accepted, with value in the units
        of the requested name (e.g. set_param("WPNAV_SPEED", 300) writes WP_SPD = 3.0 on 4.7).
        """
        fw_name, factor = translate(param_name, self.param_naming)
        if fw_name != param_name:
            print(f"Parameter {param_name} is named {fw_name} on this firmware")
        param_name, value = fw_name, value * factor
        for i in range(max_retries):
            print(f"Setting parameter {param_name} to {value} (attempt {i + 1})")  
                
            # Use MAVParmDict.mavset() instead of basic param_set_send  
            success = self.parms.mavset(self.connection, param_name, value,   
                                        retries=1, parm_type=parm_type)  
                
            if success:  
                # Verify the parameter was set correctly  
                new_value = self._fetch_param(param_name, max_retries=3)
                if new_value is not None and round(new_value, 5) == round(value, 5):  
                    print(f"Parameter {param_name} set to: {new_value}")  
                    return True  
                else:  
                    print(f"Verification failed for {param_name}: got {new_value}, expected {value}")  
                    time.sleep(0.02)  
            else:  
                print(f"Failed to set {param_name} on attempt {i + 1}")  
                time.sleep(0.02)  
            
        print(f"Failed to set parameter {param_name} after {max_retries} attempts")
        return False

    def get_autopilot_version(self, timeout=2.0):
        """Requests AUTOPILOT_VERSION and returns the firmware version as (major, minor, patch), or None."""
        self._reader.flush()
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
            0, 0, 0, 0, 0, 0
        )
        msg = self.connection.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=timeout)
        if not msg:
            print("Could not read autopilot firmware version")
            return None

        v = msg.flight_sw_version
        version = ((v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF)
        print(f"Autopilot firmware version : {version[0]}.{version[1]}.{version[2]}")
        return version

    def detect_param_naming(self):
        """Detects if the firmware uses 4.6 (WPNAV_*, cm) or 4.7 (WP_*, SI units) parameter names.

        The firmware version only decides which name is probed first, since dev builds
        labelled 4.7 may predate the rename.
        """
        expected = naming_for_version(self.ap_version) or SI
        other = LEGACY if expected == SI else SI
        for naming in (expected, other):
            if self._fetch_param(PROBE_PARAM[naming], max_retries=2, timeout=1) is not None:
                print(f"Using ArduPilot {'4.7+ (WP_*)' if naming == SI else '4.6 (WPNAV_*)'} parameter names")
                return naming
        raise RuntimeError(
            f"Could not read {PROBE_PARAM[SI]} nor {PROBE_PARAM[LEGACY]}. Is this an ArduCopter firmware?"
        )
                
    def download_all_params(self, filename = None):  
        """Method to download parameters using traditional MAVLink messages"""   
        
        # Request all parameters  
        self._reader.flush()
        self.connection.param_fetch_all()  
        
        # Collect parameter responses  
        params = {}  
        param_count = None  
        received_count = 0  
        
        print("Downloading parameters using traditional method...")  
        
        while True:  
            msg = self.connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)  
            if msg is None:  
                print(f"Timeout waiting for parameters. Received {received_count} parameters.")  
                break  
                
            # Handle both string and bytes param_id  
            if isinstance(msg.param_id, bytes):  
                param_id = msg.param_id.decode('utf-8').rstrip('\x00')  
            else:  
                param_id = str(msg.param_id).rstrip('\x00')  
                
            params[param_id] = msg.param_value  
            received_count += 1  
            
            if param_count is None:  
                param_count = msg.param_count  
                print(f"Expecting {param_count} parameters...")  
            
            print(f"Received parameter {received_count}/{param_count}: {param_id} = {msg.param_value}")  
            
            # Check if we've received all parameters  
            if received_count >= param_count:  
                break  
        
        # Save parameters to file  
        if filename is None:
            now = datetime.now()  
            datetime_string = now.strftime("%Y-%m-%d_%H-%M-%S")  
            filename = f'Params_{datetime_string}.param'  

        filename + 'param' if not filename.endswith('param') else filename

        self._save_params_to_file(params, filename)  
        print(f"Saved {len(params)} parameters to {filename}")
    
    def _save_params_to_file(self, params, filename):  
        """Save parameters in Mission Planner compatible format"""  
        with open(filename, 'w') as f:  
            # Sort parameters alphabetically for consistency  
            for param_name in sorted(params.keys()):  
                value = params[param_name]  
                # Mission Planner format: PARAM_NAME,value  
                f.write(f"{param_name},{value:.6f}\n")  



    def message_request(self, message_type: int, freq_hz: float = 10):
        """Sends a message request to the drone, allowing reception of a specific message, received at a specific rate.

        The request is only sent when the rate of this message changes. Rates stay active on the
        vehicle after the script ends, see restore_message_rates().

        Args:
            message_type (id function message): See mavlink message types that can be requested in copter mode
            freq_hz (int, optional): Desired data transmission frequency. Defaults to 10 Hz.
        """
        if self._requested_rates.get(message_type) == freq_hz:
            return
        interval_us = int(1e6 / freq_hz)  # Interval in microseconds
        self.connection.mav.command_long_send(
            self.connection.target_system,  # Target system ID
            self.connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # Command to set message interval
            0,  # Confirmation
            message_type,  # Message ID
            interval_us,  # Interval in microseconds
            0,
            0,
            0,
            0,
            0,  # Unused parameters
        )
        self._requested_rates[message_type] = freq_hz

    def restore_message_rates(self):
        """Puts back the default rate of every message requested by this Zenmav instance."""
        for message_type in list(self._requested_rates):
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_type,
                0,  # 0 : default rate
                0, 0, 0, 0, 0,
            )
        self._requested_rates.clear()

    def set_mode(self, mode: str, max_retries = 3):
        """Allows easy mode selection from its string

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            mode (str): Mode identification by letters
        """
        connection = self.connection
        mode_map = connection.mode_mapping()
        if mode_map is None:
            raise RuntimeError("Vehicle type unknown : no heartbeat received from the autopilot")
        mode_id = mode_map[mode]  # Conversion of mode to its id

        for i in range(max_retries):
            self._reader.flush()  # Only a heartbeat sent after the request can confirm the mode
            connection.mav.set_mode_send(
                connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )

            heartbeat = self._wait_vehicle_heartbeat(timeout=1.0)
            if heartbeat and connection.flightmode.upper() == mode.upper():
                print(f"Mode successfully changed to {mode}")
                break
            else:
                print(f'FAILED TO CHANGE MODE TO {mode}')
            


    def arm(self):
        """Arms the drone"""
        connection = self.connection
        self.message_request(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, freq_hz=1)
        armable = False

        while self.connection.recv_match(type="SYS_STATUS", blocking=False, timeout=2):
            time.sleep(0.001)

        while not armable:
            sys_status = connection.recv_match(
                type="SYS_STATUS", blocking=True, timeout=5
            )
            if sys_status:
                prearm_bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
                # Check if pre-arm checks are healthy
                if sys_status.onboard_control_sensors_health & prearm_bit:
                    print("Vehicle is armable - pre-arm checks passed")
                    armable = True
                else:
                    print("Vehicle is NOT armable - pre-arm checks failed")
                    time.sleep(0.2)
            else:
                print("ERROR : ARMING FAILURE PLEASE VERIFY SYSTEM")
                break

        # Arm the vehicle
        print("Arming motors...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )

       # Wait for arming confirmation
        connection.motors_armed_wait()
        print("Motors armed!")

    def takeoff(self, altitude: float = 10.0, threshold = 1, while_moving=None):
        """Makes the drone take off. Requires 'GUIDED' mode, and the drone to be armed.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            altitude (int, optional): Drone altitude in m of height relative to origin. Defaults to 10.
        """
        # Takeoff
        current_pos = self.get_global_pos()
        above_home = current_pos.copy()
        above_home.alt += altitude

        print(f"Taking off to {altitude} meters...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )
        print("Waiting for takeoff...")
        while self.is_near_waypoint(self.get_global_pos().alt, altitude, threshold=threshold) == False:
            if while_moving is not None:
                while_moving()
            else:
                pass
        
        self.global_target(above_home) #Ensure the drone starts directly above takeoff point


    def guided_arm_takeoff(self, height: float = 20.0):
        """Allows quick connection, arming the drone and taking off

        Args:
            ip (str, optional): See connect documentation for more information
            height (int, optional): Drone takeoff height. Defaults to 20.
        """

        # Set mode to GUIDED
        self.set_mode("GUIDED")

        self.arm()

        self.takeoff(height)

    def RTL(self, while_moving=None):
        """Sends an RTL command (return to launch). Waits for the drone to land, once landed, the drone is disarmed and the connection closes automatically, indicating the end of the mission.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
        """
        connection = self.connection
        print("Returning to launch...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        while self.get_global_pos().alt > 1.0:
            if while_moving is not None:
                while_moving()
            else:
                pass
        else:
            connection.motors_disarmed_wait()
            print("Landed and motors disarmed!")

            self._close_link()
            print("Connection closed. Mission Finished")

    def close_all_connections(self):
        """Closes the drone link and, in GCS mode, stops the relay and its TCP servers."""
        self._close_link()
        if self.relay is not None:
            self.relay.close()
            self.relay = None

    def _close_link(self):
        """Stops the fence monitor, restores message rates, stops the heartbeat and closes the connection."""
        if self._link_closed:
            return
        self._link_closed = True
        if hasattr(self, "limits"):
            self.limits.stop_breach_monitor(timeout=1.0)
        try:
            self.restore_message_rates()
        except Exception:
            pass
        self._heartbeat_stop.set()
        self.connection.close()

    def insert_coordinates_to_csv(self, file_path: str, waypoint: wp, description = True):
        """
        Inserts coordinates into a CSV file. If the file doesn't exist, it creates one with a header.

        Parameters:
            file_path (str): Path to the CSV file.
            coordinates (tuple): (latitude, longitude) coordinates.

        Example:
            insert_coordinates_to_csv("coordinates.csv", [(45.5017, -73.5673), (40.7128, -74.0060)])
        """
        # Check if the file exists
        try:
            with open(file_path, mode="r") as file:
                file_exists = True
        except FileNotFoundError:
            file_exists = False

        # Open the file in append mode
        with open(file_path, mode="a", newline="") as file:
            writer = csv.writer(file)

            # If the file doesn't exist, write the header
            if not file_exists:
                if waypoint.frame == "global":
                    writer.writerow(["Latitude", "Longitude", "Altitude", "Name"])
                elif waypoint.frame == "local":
                    writer.writerow(["North", "East", "Down", "Name"])
                elif waypoint.frame == "base_link":
                    writer.writerow(["Forward", "East", "Down" , "Name"])
            
            writer.writerow([waypoint.coordinates[0], waypoint.coordinates[1] ,waypoint.coordinates[2], waypoint.name])

    def spiral_scan(
        self,
        detection_width: float = 10.0,
        altitude: float = 10.0,
        scan_radius: float = 100.0,
        safety_margin: float = 0.0,
        center: list = None,
    ):
        """Allows generating points to follow in order to scan a circular area, by performing a spiral. Also measures the time taken to complete the entire scan.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            detection_width (int, optional): Horizontal distance over which the drone can detect an emitter. Defaults to 10.
            altitude (int, optional): Relative height of home at which to perform the scan. Defaults to 10.
            scan_radius (int, optional): Radius of the area to scan. Defaults to 100.
            safety_margin (int, optional): Additional distance to radius to compensate for initial positioning error. Defaults to 0.
            center (local_pos, optional): Local coordinates of the scan center. If left at None, take the initial position when the function is called.
        """
        if center is None:
            pos = self.get_local_pos()
        else:
            pos = center

        spacing = detection_width
        number_of_turns = scan_radius / spacing

        scan_radius += safety_margin

        # Spiral parameters
        theta_spiral = np.linspace(0, 2 * np.pi * number_of_turns, 100)
        b = spacing / (2 * np.pi)
        r_spiral = b * theta_spiral
        x_spiral = r_spiral * np.cos(theta_spiral) + pos[0]
        y_spiral = r_spiral * np.sin(theta_spiral) + pos[1]

        start_time = time.time()

        for i in range(len(x_spiral)):
            waypoint = wp(x_spiral[i], y_spiral[i], -altitude, frame = "local")
            self.global_target(waypoint)

        total_time = time.time() - start_time
        print("SCAN FINISHED")
        print(f"Total time taken : {total_time:.2f}")

    def rectilinear_scan(
        self,
        detection_width: float = 10.0,
        altitude: float = 10.0,
        scan_radius: float = 100.0,
        safety_margin: float = 0.0,
        center: wp = None,
    ):
        """Allows generating points to follow in order to scan a circular area, by performing a rectilinear pattern. Also measures the time taken to complete the entire scan.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            detection_width (int, optional): Horizontal distance over which the drone can detect an emitter. Defaults to 10.
            altitude (int, optional): Relative height of home at which to perform the scan. Defaults to 10.
            scan_radius (int, optional): Radius of the area to scan. Defaults to 100.
            safety_margin (int, optional): Additional distance to radius to compensate for initial positioning error. Defaults to 0.
            center (local_pos, optional): Local coordinates of the scan center. If left at None, take the initial position when the function is called.
        """
        if center is None:
            pos = self.get_local_pos()
        else:
            pos = center

        e = detection_width
        radius = scan_radius
        safety_margin = 0
        radius += safety_margin
        x = []
        y = []
        high = True
        n_passes = int(2 * radius / e)
        for n in range(n_passes):
            w = e * (1 / 2 + n)
            h = np.sqrt(radius ** 2 - (radius - w) ** 2)
            if high:
                y.append(-radius + w)
                x.append(h)
                y.append(-radius + w)
                x.append(-h)
                high = False
            else:
                y.append(-radius + w)
                x.append(-h)
                y.append(-radius + w)
                x.append(h)
                high = True

        start_time = time.time()

        x, y = y, x

        for i in range(len(x)):
            self.global_target(wp(x[i] + pos.N, y[i] + pos.E, -altitude, frame = "local"))

        total_time = time.time() - start_time
        print("SCAN FINISHED")
        print(f"Total time: {total_time:.2f} seconds")

    def auto_flip(self, initial_throttle: int = 1750):
        """Performs an auto flip maneuver by setting the drone to 'FLIP' mode after a short delay at the set throttle value, and then returning to 'GUIDED' mode and returns to initial position.

        Args:
            initial_throttle (int, optional): Throttle value to keep 0.5 s before the flip. Defaults to 1750.
        """
        input("Press Enter...")
        initial_pos = self.get_global_pos()
        self.set_mode("ALT_HOLD")
        flipped = False
        start_time = time.time()
        while True:

            if (time.time() - start_time > 0.5) and (not flipped):
                self.set_mode("FLIP")
                flipped = True
            elif time.time() - start_time > 3:
                print("Exiting loop after 3 seconds.")
                break
            else:
                self.connection.mav.rc_channels_override_send(
                    self.connection.target_system,  # target_system
                    self.connection.target_component,  # target_component
                    65535,  # chan1_raw (UINT16_MAX = ignore)
                    65535,  # chan2_raw (UINT16_MAX = ignore)
                    initial_throttle,  # chan3_raw (throttle override to 1500μs)
                    65535,  # chan4_raw (UINT16_MAX = ignore)
                    65535,  # chan5_raw (UINT16_MAX = ignore)
                    65535,  # chan6_raw (UINT16_MAX = ignore)
                    65535,  # chan7_raw (UINT16_MAX = ignore)
                    65535,  # chan8_raw (UINT16_MAX = ignore)
                )

        self.set_mode("GUIDED")
        self.global_target(initial_pos)

    def get_battery(self):
        """Fetches the current battery status from the drone.

        Returns:
            dict: A dictionary containing battery voltage, current, and remaining percentage.
        """
        self.message_request(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, freq_hz=10)

        while self.connection.recv_match(type="SYS_STATUS", blocking=False, timeout=2):
            pass

        sys_status = self.connection.recv_match(
            type="SYS_STATUS", blocking=True, timeout=2
        )
        if sys_status:
            print(f"Battery Voltage: {sys_status.voltage_battery/1000} V")
            print(f"Battery Current: {sys_status.current_battery/100} A")
            return battery(
                sys_status.voltage_battery / 1000.0, sys_status.current_battery / 100.0
            )  # Convert to volts and amps
        
    def get_attitude(self):

        self.message_request(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, freq_hz=100)

        while self.connection.recv_match(type="ATTITUDE", blocking=False, timeout=2):
            pass

        attitude = self.connection.recv_match(
            type="ATTITUDE", blocking=True, timeout=2
        )
        if attitude:
            yaw = attitude.yaw*180/np.pi
            if yaw < 0:
                yaw += 360
            return (attitude.roll*180/np.pi, attitude.pitch*180/np.pi, yaw )

    def rc_override(self, channel_values: dict):
        """
        Override one or more RC input channels.

        This simulates receiver PWM inputs. Provide only the channels you want
        to override; any channel you omit is sent as UINT16_MAX (65535), which
        instructs the flight controller to ignore that channel and keep using
        the real RC input.

        Args:
            channel_values (dict[str, int]): Mapping of channel names to PWM
                pulse widths in microseconds. Keys must be 'ch1'..'ch8'.
                Each value must be an integer in the range [1000, 2000].
                Example: {'ch3': 1500, 'ch7': 1800}

        Notes:
            • Overrides must be refreshed periodically (≈5–10 Hz). If you stop
            sending them, the FC will revert to normal RC inputs.
            Channel mapping note

            • Standard ELRS / EdgeTX (Mode 2) uses AETR:
            ch1=Roll (Aileron), ch2=Pitch (Elevator), ch3=Throttle, ch4=Yaw (Rudder).
            • Aux channels (common on ArduPilot):
            ch5=Flight mode switch, ch6=Tuning/knob, ch7/ch8=User-assigned (RCx_OPTION).
            • Some radios use TAER instead:
            ch1=Throttle, ch2=Roll, ch3=Pitch, ch4=Yaw.


        Returns:
            None
        """
        ch = [int(channel_values.get(f"ch{i}", 65535)) for i in range(1, 9)]

        self.connection.mav.rc_channels_override_send(
            self.connection.target_system, self.connection.target_component, *ch
        )
    
    def set_home(self, timeout=2.0):  
        """  
        Set self.home to the actual HOME_POSITION instead of EKF origin.  
        Returns True on success.  
        """  
        self._reader.flush()
        # Request HOME_POSITION message (ID 242)  
        self.connection.mav.command_long_send(  
            self.connection.target_system,   
            self.connection.target_component,  
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,   
            0,  
            242,  # HOME_POSITION message ID  
            0, 0, 0, 0, 0, 0  
        )  
        
        msg = self.connection.recv_match(type="HOME_POSITION", blocking=True, timeout=timeout)  
        if not msg:  
            return False  
    
        lat = msg.latitude / 1e7  
        lon = msg.longitude / 1e7  
        alt = msg.altitude / 1000.0  # Convert from mm to meters  
        self.home = wp(lat, lon, alt, frame="global")  
        return True

    
    def orbit(self, center, radius, speed, clockwise = True, N_turns = 1, force = False, initial_position_threshold = 0.1, radius_tolerance = 1):
        if center.frame == "global":
            center = self.convert_to_local(center)
        if speed**2/radius > 1.5 and not force:
            print('WARNING : HIGH CENTREPIDAL ACCELERATION, PRONE TO ERROR')
            print('SET force = True TO PROCEED, ABORTING')
            return None
        pos = self.get_local_pos()
        posN, posE = pos.N, pos.E

        dx, dy = posN - center.N, posE - center.E
        d = (dx*dx + dy*dy) ** 0.5
        if d == 0:
            return center.N + radius, center.E          # arbitrary direction if you're at the center
        k = radius / d
        N_point, E_point = center.N + k*dx, center.E + k*dy
        E_error, N_error = center.E - E_point,  center.N - N_point
        hdg_init = atan2(E_error, N_error)*180/3.141592

        circumference = 2*np.pi*radius
        time_for_loop = circumference/speed
        rate = 360/time_for_loop

        if clockwise :
            speed = -speed
        else:
            rate = -rate

        radius_pid = PIDController(3.5, 0, 2)


        above_target = wp(N_point, E_point, center.D, frame = "local")
        initial_wp_radius = self.get_param('WP_RADIUS_M')
        self.set_param('WP_RADIUS_M', initial_position_threshold/2)
        self.local_target(above_target, acceptance_radius= initial_position_threshold, heading=hdg_init)
        time.sleep(2)
        actual_pos = self.get_local_pos()
        actual_x, actual_y = actual_pos.N, actual_pos.E
        E_error, N_error = center.E - actual_y,  center.N - actual_x
        hdg = atan2(E_error, N_error)*180/3.141592
        if hdg < 0:
            hdg += 360

        self.yaw_target(hdg)
        error = round(self.get_attitude()[2],0) - round(hdg,0)
        while error > 1:
            yaw = round(self.get_attitude()[2],0)
            error =  yaw - round(hdg,0)
            time.sleep(0.01)
        time.sleep(1)

        
        
        first = True
        start_time = time.time()
        while time.time() - start_time < N_turns*time_for_loop:
            if first :
                first = False
                self.speed_target((0,-speed,0) , yaw_rate=rate)
                F_speed = 0
            else:
                actual_pos = self.get_local_pos()
                actual_x, actual_y = actual_pos.N, actual_pos.E
                E_error, N_error = center.E - actual_y,  center.N - actual_x
                distance = np.linalg.norm((E_error, N_error))
                difference = radius - distance
                if difference > radius_tolerance:
                    print('ORBIT TRACKING FAILED, ABORTING')
                    break
                F_speed = radius_pid.compute(difference, time.time() - last_time)

            self.speed_target((-F_speed,speed,0) , yaw_rate=rate)
            last_time = time.time()

        self.speed_target((0,0,0), yaw_rate=0)
        self.set_param('WP_RADIUS_M', initial_wp_radius)


class battery:
    def __init__(self, voltage: float, current: float):
        self.voltage = voltage
        self.current = current



class PIDController():
    def __init__(self, kp, ki, kd, max_output = 3.0):  # Une norme de 3.0 m/s est le max pour vitesses xy envoyées au drone
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Clamp output to max value
        return max(min(output, self.max_output), -self.max_output)

