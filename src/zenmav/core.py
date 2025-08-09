from pymavlink import mavutil
import time
import numpy as np
import csv
from math import atan2
from geopy.distance import distance
from geopy import Point
import threading 
import select

class Zenmav():
    def __init__(self, ip: str = 'tcp:127.0.0.1:5762' , baud = None, gps_thresh : float= None, GCS = False, tcp_ports = [14551]):
        '''Initializes the Zenmav class, allowing connection to a drone via MAVLink protocol.'''
        self = self
        self.last_message_req = None
        self.gps_thresh = gps_thresh  # GPS threshold in meters

        if GCS:
            ip = self.split_connections(ip, tcp_ports)

        self.connect(ip, baud)

        if self.gps_thresh is not None:

            nav_thresh = self.get_param('WPNAV_RADIUS')/100
            print(f'Nav_thresh : {nav_thresh}')
            print(f'GPS_thresh : {self.gps_thresh}')

            if nav_thresh > self.gps_thresh:
                while True:
                    print(f"WARNING : Zenmav threshold {self.gps_thresh} is less than the AP nav threshold {nav_thresh}.")


            self.home = self.get_global_pos()
            ref_point = Point(self.home[0], self.home[1])
            point_north = distance(meters=self.gps_thresh).destination(ref_point, bearing=0)
            self.lat_thresh = abs(point_north.latitude - ref_point.latitude)

            point_east = distance(meters=self.gps_thresh).destination(ref_point, bearing=90)
            self.lon_thresh = abs(point_east.longitude - ref_point.longitude)

    def split_connections(self, ip : str, tcp_ports : list):
        self.connections = []  
        self.connections.append(mavutil.mavlink_connection(ip))
        self.connections.append(mavutil.mavlink_connection('tcpin:0.0.0.0:14550'))  
        for port in tcp_ports:
            self.connections.append(mavutil.mavlink_connection(f'tcpin:0.0.0.0:{port}'))
        
        forwarding_thread = threading.Thread(target=self.message_forwarder, daemon=True)  
        forwarding_thread.start()  
        ip = 'tcp:127.0.0.1:14550'
        return ip

    def message_forwarder(self):  
        while True:  
            # Separate UDP and TCP connections  
            udp_connections = [conn for conn in self.connections if isinstance(conn, mavutil.mavudp)]  
            tcp_connections = [conn for conn in self.connections if hasattr(conn, "fd") and conn.fd is not None]  
            
            # Handle TCP connections with select  
            if tcp_connections:  
                fd_to_conn = {conn.fd: conn for conn in tcp_connections}  
                ready_fds, _, _ = select.select(fd_to_conn.keys(), [], [], 0.01)  # Short timeout  
                for fd in ready_fds:  
                    conn = fd_to_conn[fd]  
                    try:  
                        msg = conn.recv_match(blocking=False)  
                        if msg:  
                            buf = msg.get_msgbuf()  
                            for other in self.connections:  
                                if other is not conn:  
                                    other.write(buf)  
                    except (TypeError, AttributeError) as e:  
                        # Skip corrupted messages that cause state issues  
                        continue 
                    
            if udp_connections:
                # Handle UDP connections separately  
                for conn in udp_connections:  
                    try:  
                        msg = conn.recv_match(blocking=False)  
                        if msg:  
                            buf = msg.get_msgbuf()  
                            for other in self.connections:  
                                if other is not conn:  
                                    other.write(buf)  
                    except (TypeError, AttributeError) as e:  
                        # Skip corrupted messages that cause state issues  
                        continue 
                
                time.sleep(0.001)  # Small delay to prevent CPU spinning

    def connect(self, ip_address : str ='tcp:127.0.0.1:5762', baud : int = None):
        """Enables easy connection to the drone, and waits for heartbeat to ensure a live communication. Only call this function once it init, should NOT be run outside of init.

        Args:
            ip_address (str, optional): IP address for connection.
                Sitl simulation : 'tcp:127.0.0.1:5762' .
                Real connection : 'udp:<ip_ubuntu>:14551' (Ensure the antenna signal is properly transmitted on this port and UDP communication is allocated between windows-ubuntu).
                Zenith Siyi connexion : 'udpout:192.168.144.12:19856'
        Returns:
            None
        """

        # Create the self.connection
        # Establish connection to MAVLink
        if baud is not None:
            self.connection = mavutil.mavlink_connection(ip_address, baud=baud)
        else:
            self.connection = mavutil.mavlink_connection(ip_address)
        print('Waiting for heartbeat...')
        self.connection.mav.heartbeat_send(  
            mavutil.mavlink.MAV_TYPE_GCS,  
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,  
            0, 0, 0  
        )  
        self.connection.wait_heartbeat()
        print("Heartbeat received!")
    
    def global_target(self, wp : list, acceptance_radius : float=8e-6, while_moving  =None, wait_to_reach : bool=True):
            """Sends a movement command to the drone for a specific global GPS coordinate.

            Args:
                wp (tuple): Target waypoint as (latitude, longitude, altitude in meters).
                acceptance_radius (float, optional): Distance at which the target is considered reached. Defaults to 5 meters. Deprecated, use gps_thresh in Zenmav init.
                while_moving (function, optional) to execute while the drone is in transit.
                wait_to_reach (bool, optional): Whether to wait for the drone to reach the target before proceeding.
            """
            connection = self.connection
            print(wp)

            # Send a MAVLink command to set the target global position
            connection.mav.set_position_target_global_int_send(
                0,  # Timestamp in milliseconds
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Global frame with relative altitude
                0b100111111000,  # Position mask
                int(wp[0] * 1e7),  # Latitude in degrees * 1e7
                int(wp[1] * 1e7),  # Longitude in degrees * 1e7
                wp[2],  # Altitude in meters (relative to home)
                0, 0, 0,  # No velocity set
                0, 0, 0,  # No acceleration set
                0, 0  # No yaw or yaw rate
            )

            gps = True if self.gps_thresh is not None else False

            if wait_to_reach:
                # Wait for the waypoint to be reached
                print("Waiting for waypoint to be reached...")
                while not self.is_near_waypoint(self.get_global_pos(), wp, threshold=acceptance_radius, gps=gps):
                    if while_moving is not None:
                        while_moving()
                    else:
                        pass
                else:
                    print("Waypoint reached!")

    def is_near_waypoint(self, actual : list, target: list, threshold : float = 2., gps : bool = False):
        """Returns True if the distance between the drone and the target is < threshold. Else False.

        Args:
            actual (list): Current drone position (Local NED coordinates : [N, E, -Z])
            target (list): Target position to compare (Local NED coordinates : [N, E, -Z])
            threshold (float, optional): Distance at which to return True. Defaults to 2.

        Returns:
            bool: True if drone is close enough, False otherwise
        """
        
        if gps:
            return (abs(actual[0] - target[0]) <= self.lat_thresh) and (abs(actual[1] - target[1]) <= self.lon_thresh)
        else:
            return np.linalg.norm(np.array(actual) - np.array(target)) < threshold


    def get_local_pos(self, frequency_hz : int=60):
        """Allows to get the local position, and makes a request to get the data at the desired frequency.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            frequency_hz (int, optional): Frequency of data requests. Defaults to 60.

        Returns:
            Position (list): Position in local NED coordinate system : [N, E, -Z])
        """


        self.message_request(message_type=mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, freq_hz=frequency_hz)

        while self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=1):
            pass  # Discard old messages

        # Loop to receive the most recent message
        while True:
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True) 
            if msg and msg.get_type() == "LOCAL_POSITION_NED":
                print(f"Position: X = {msg.x} m, Y = {msg.y} m, Z = {msg.z} m")
                return [msg.x, msg.y, msg.z]
            # Reduce busy-waiting and ensure responsiveness



    def get_global_pos(self, time_tag : bool =False, heading : bool = False):
        """Gets the current global position of the drone in GPS coordinates (latitude, longitude, altitude). Optionally includes a time tag and heading.

        Args:
            time_tag (bool, optional): Include the timestamp of the data. Defaults to False.
            heading (bool, optional): Include the heading as last value in the tuple return. Defaults to False.

        Returns:
            tuple : [timestamp, latitude, longitude, altitude, heading] 
            timestamp and heading are optional, depending on the parameters.
        """
        connection = self.connection
        self.message_request(message_type=mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, freq_hz=60)

        while connection.recv_match(type="GLOBAL_POSITION_INT", blocking=False, timeout=1):
            pass  # Discard old messages

        if time_tag == False:
            # Fetch the current global position
            while True:
                msg = connection.recv_match(blocking=True)
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    # Extract latitude, longitude, and relative altitude
                    lat = msg.lat / 1e7  # Convert from int32 to degrees
                    lon = msg.lon / 1e7  # Convert from int32 to degrees
                    alt = msg.relative_alt / 1000.0  # Convert from mm to meters (relative altitude)
                    hdg = msg.hdg/100

                    print(f"Position: Lat = {lat}°, Lon = {lon}°, Alt = {alt} meters, hdg = {hdg}")
                    
                    if heading:
                        pos = lat, lon, alt, hdg
                    else:
                        pos = lat, lon, alt
                    return pos
        else:
            # Fetch the current global position with time tag
            while True:
                msg = connection.recv_match(blocking=True)
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    # Extract latitude, longitude, and relative altitude
                    lat = msg.lat / 1e7  # Convert from int32 to degrees
                    lon = msg.lon / 1e7  # Convert from int32 to degrees
                    alt = msg.relative_alt / 1000.0  # Convert from mm to meters (relative altitude)
                    hdg = msg.hdg/100

                    timestamp = msg.time_boot_ms / 1000.0

                    if heading:
                        pos = timestamp, lat, lon, alt, hdg
                    else:
                        pos = timestamp, lat, lon, alt
                    return pos

    def get_rc_value(self, channel : int):
        """
        Retrieve the raw value of an RC channel.
        
        Args:
            connection: MAVLink connection object.
            channel (int): Channel number to read (1-18).

        Returns:
            int: Raw RC channel value (1000 - 2000).
        """

        self.message_request(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, freq_hz=60)

        while self.connection.recv_match(type="RC_CHANNELS", blocking=False):
            pass  # Discard old messages
        
        while True:
            msg = self.connection.recv_match(type="RC_CHANNELS", blocking=True)
            if msg and msg.get_type() == "RC_CHANNELS":
                # Channel values are indexed from 1 to 18
                if 1 <= channel <= 18:
                    value = getattr(msg, f'chan{channel}_raw', None)
                    if value is not None:
                        #print(f"RC Channel {channel} Value: {value}")
                        return value
                    else:
                        print(f"Channel {channel} not available in the message.")
                        return None
    def get_param(self, param_name : str):
        """Fetches a specific parameter from the drone.

        Args:
            param_name (str): The name of the parameter to fetch.

        Returns:
            float: The value of the requested parameter.
        """
        self.connection.param_fetch_one(param_name)  
        print(f"Requesting parameter: {param_name}")
        # Wait for the parameter response  
        msg = self.connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)  
        if msg and msg.param_id == param_name:  
            param_value = msg.param_value
            print(f"Parameter {param_name}: {param_value}")
            return param_value
        
    def set_param(self, param_name : str, value : float):
        """Sets a specific parameter on the drone.

        Args:
            param_name (str): The name of the parameter to set.
            value (float): The value to set for the parameter.
        """
        print(f"Setting parameter {param_name} to {value}")
        self.connection.param_set_send(param_name, value)  
        # Wait for confirmation  
        msg = self.connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)  
        if msg and msg.param_id == param_name:  
            print(f"Parameter {param_name} set to: {msg.param_value}")

    def message_request(self, message_type : str, freq_hz : int =10):
        """Sends a message request to the drone, allowing reception of a specific message, received at a specific rate.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            message_type (id function message): See mavlink message types that can be requested in copter mode
            freq_hz (int, optional): Desired data transmission frequency. Defaults to 10 Hz.
        """
        if message_type != self.last_message_req:
            interval_us = int(1e6 / freq_hz)  # Interval in microseconds
            # Send the command to set the message interval
            self.connection.mav.command_long_send(
                self.connection.target_system,  # Target system ID
                self.connection.target_component,  # Target component ID
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # Command to set message interval
                0,  # Confirmation
                message_type,  # Message ID for GLOBAL_POSITION_INT
                interval_us,  # Interval in microseconds
                0,
                0,
                0,
                0,
                0,  # Unused parameters
            )
            self.last_message_req = message_type



    def set_mode(self, mode : str):
        """Allows easy mode selection from its string

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            mode (str): Mode identification by letters
        """
        connection = self.connection
        mode_id = connection.mode_mapping()[mode] #Conversion of mode to its id
        connection.mav.set_mode_send(connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        print(f"Setting mode to {mode}...")


    def arm(self):
        """Arms the drone
        """
        connection = self.connection
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


    def takeoff(self, altitude : float = 10., while_moving  = None):
        """Makes the drone take off. Requires 'GUIDED' mode, and the drone to be armed.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            altitude (int, optional): Drone altitude in m of height relative to origin. Defaults to 10.
        """
        # Takeoff
        connection = self.connection

        print(f"Taking off to {altitude} meters...")
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
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
        while self.is_near_waypoint(self.get_local_pos()[2], -altitude) == False:
            if while_moving is not None:
                while_moving()
            else:
                pass


    def guided_arm_takeoff(self, height : float = 20.):
        """Allows quick connection, arming the drone and taking off

        Args:
            ip (str, optional): See connect documentation for more information
            height (int, optional): Drone takeoff height. Defaults to 20.
        """

        # Set mode to GUIDED
        self.set_mode( "GUIDED")

        self.arm()

        self.takeoff(height)

    def convert_to_global(self, local_pos : list, reference_point : list=None):
        if reference_point is None:
            reference_point = self.home
        """Converts local NED coordinates to global GPS coordinates.

        Args:
            local_pos (list): Local position in NED system [N, E].

        Returns:
            tuple: Global GPS position (latitude, longitude, altitude).
        """
        x, y = local_pos
        point_north = distance(meters=y).destination(reference_point, bearing=0)
        final_point = distance(meters=x).destination(point_north, bearing=90)
        return [final_point.latitude, final_point.longitude]



    def local_target(self, wp : list, acceptance_radius: float = 5., while_moving = None, turn_into_wp : bool = False):
        """Allows easy sending of a drone movement command to local coordinates in NED system.

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            wp (list): list of coordinates in local coordinate system [N, E, D] (YES POSITIVE ALTITUDE = NEGATIVE)
            while_moving (function) : Thing to do while waiting for wp to be reached
            acceptance_radius (int, optional): Distance at which the drone considers the target reached. Defaults to 5.
        """
        
        connection = self.connection

        yaw_angle = 0
        if turn_into_wp:
            actual_pos = self.get_local_pos()
            actual_x, actual_y = actual_pos[0], actual_pos[1]
            yaw_angle = atan2(wp[0] - actual_x, wp[1] - actual_y)
        

        connection.mav.set_position_target_local_ned_send(
            0,  # Time in milliseconds
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b10011111000,  # Position mask
            wp[0],
            wp[1],
            wp[2],  # X (North), Y (East), Z (Down)
            0,
            0,
            0,  # No velocity
            0,
            0,
            0,  # No acceleration
            yaw_angle,
            0,  # No yaw or yaw rate
        )

        # Wait for the waypoint to be reached
        print("Waiting for waypoint to be reached...")
        while not self.is_near_waypoint(self.get_local_pos(), wp, threshold=acceptance_radius):
            if while_moving is not None:
                while_moving()
            else:
                pass
        else:
            print("Waypoint reached!")

    def speed_target(self, wp:list, yaw_rate : float=0.):
        yaw_rate = yaw_rate * np.pi / 180  # Convert degrees to radians
        """Allows easy sending of a drone speed command in its reference system (Forward, right, down).

        Args:
            connection (mavlink connection): Connection to the drone, often called master or connection
            wp (list): list of coordinates in system of [Forward, Right, Down] (YES POSITIVE ALTITUDE = NEGATIVE)
            yaw_rate (float, optional): Speed of drone rotation around its vertical axis. Defaults to 0. IN degrees per second.
        """
        
        connection = self.connection
        
        connection.mav.set_position_target_local_ned_send(
            0,  # Time in milliseconds
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b010111000111,  # Speed mask
            0,
            0,
            0,   #X Front, Y Right, Z Down
            wp[0],
            wp[1],
            wp[2],  # No velocity
            0,
            0,
            0,  # No acceleration
            0,  # No yaw or
            yaw_rate,   #yaw rate
        )

        # Wait for the waypoint to be reached
        print(f"Speed command of {wp} m/s")


    def RTL(self, while_moving  = None):
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

        while self.get_local_pos()[2] > - 0.5:
            if while_moving is not None:
                while_moving()
            else:
                pass
        else:
            connection.motors_disarmed_wait()
            print("Landed and motors disarmed!")

            connection.close()
            print("Connection closed. Mission Finished")



    def insert_coordinates_to_csv(self, file_path : str, coordinates : tuple, desc : str):
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
            with open(file_path, mode='r') as file:
                file_exists = True
        except FileNotFoundError:
            file_exists = False
        
        # Open the file in append mode
        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            
            # If the file doesn't exist, write the header
            if not file_exists:
                writer.writerow(["Latitude", "Longitude", "Description"])

            writer.writerow([coordinates[0], coordinates[1], desc])

    def append_description_to_last_line(file_path : str, description : str):
        """
        Appends a description to the last line of a CSV file. The description is added in a new column.
        
        Parameters:
            file_path (str): Path to the CSV file.
            description (str): The description to append.
            
        Example:
            append_description_to_last_line("coordinates.csv", "City Center")
        """
        # Read the existing content of the CSV file
        rows = []
        try:
            with open(file_path, mode='r', newline='') as file:
                reader = csv.reader(file)
                rows = list(reader)
        except FileNotFoundError:
            print("Error: The file does not exist.")
            return
        
        # Check if there's at least one row (after header)
        if len(rows) <= 1:
            print("Error: No data rows to update.")
            return
        
        # Append the description to the last row
        last_row = rows[-1]
        last_row.append(description)
        
        # Write the updated rows back to the file
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(rows)


    def spiral_scan(self, detection_width : float = 10., altitude : float = 10., scan_radius : float = 100., safety_margin : float = 0., center :list = None):
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
        theta_spiral = np.linspace(0, 2 * np.pi*number_of_turns, 100)
        b = spacing/(2*np.pi)
        r_spiral = b * theta_spiral
        x_spiral = r_spiral * np.cos(theta_spiral) + pos[0]
        y_spiral = r_spiral * np.sin(theta_spiral) + pos[1]

        start_time = time.time()

        for i in range(len(x_spiral)):
            wp = [x_spiral[i], y_spiral[i], -altitude]
            self.local_target(wp, acceptance_radius=10)

        total_time = time.time() - start_time
        print("SCAN FINISHED")
        print(f"Total time taken : {total_time:.2f}")


    def rectilinear_scan(self, detection_width : float = 10., altitude : float = 10., scan_radius : float = 100., safety_margin : float = 0., center :list = None):
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
        n_passes = int(2*radius/e)
        for n in range(n_passes):
            w = e*(1/2 + n)
            h = np.sqrt(radius**2 - (radius - w)**2)
            if high:
                x.append(-radius + w)
                y.append(h)
                x.append(-radius + w)
                y.append(-h)
                high = False
            else:
                x.append(-radius + w)
                y.append(-h)
                x.append(-radius + w)
                y.append(h)
                high = True
        start_time = time.time()

        for i in range(len(x)):
            wp = [x[i] + pos[0], y[i] + pos[1], -altitude]
            self.local_target(wp, acceptance_radius=3)

        total_time = time.time() - start_time
        print("SCAN FINISHED")
        print(f"Total time: {total_time:.2f} seconds")

    def auto_flip(self, initial_throttle : int = 1750):
        """ Performs an auto flip maneuver by setting the drone to 'FLIP' mode after a short delay at the set throttle value, and then returning to 'GUIDED' mode and returns to initial position.

        Args:
            initial_throttle (int, optional): Throttle value to keep 0.5 s before the flip. Defaults to 1750.
        """
        input("Press Enter...")
        initial_pos = self.get_global_pos()
        self.set_mode('ALT_HOLD')
        flipped = False
        start_time = time.time()
        while True:
             
            if (time.time() - start_time > 0.5) and (not flipped):
                self.set_mode('FLIP')
                flipped = True
            elif time.time() - start_time > 3:
                print("Exiting loop after 3 seconds.")
                break
            else:
                self.connection.mav.rc_channels_override_send(  
                self.connection.target_system,    # target_system  
                self.connection.target_component, # target_component  
                65535,  # chan1_raw (UINT16_MAX = ignore)  
                65535,  # chan2_raw (UINT16_MAX = ignore)   
                initial_throttle,   # chan3_raw (throttle override to 1500μs)  
                65535,  # chan4_raw (UINT16_MAX = ignore)  
                65535,  # chan5_raw (UINT16_MAX = ignore)  
                65535,  # chan6_raw (UINT16_MAX = ignore)  
                65535,  # chan7_raw (UINT16_MAX = ignore)  
                65535   # chan8_raw (UINT16_MAX = ignore)  
            ) 

        self.set_mode('GUIDED')
        self.global_target(initial_pos)