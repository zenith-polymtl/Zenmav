from pymavlink import mavutil
import time
import numpy as np
import csv
from math import atan2
from geopy.distance import distance
from geopy import Point

class Zenmav():
    def __init__(self, gps_thresh= None, ip = 'tcp:127.0.0.1:5762' , gps = True):
        self = self
        self.last_message_req = None

        self.connect(ip)
        if gps:
            self.home = self.get_global_pos()
            print(f"Home position: {self.home}")

        if gps_thresh is not None:
            gps_pos = self.get_global_pos()
            ref_point = Point(gps_pos[0], gps_pos[1])
            point_north = distance(meters=gps_thresh).destination(ref_point, bearing=0)
            self.lat_thresh = abs(point_north.latitude - ref_point.latitude)

            # 1.7 meters East (Longitude axis)
            point_east = distance(meters=gps_thresh).destination(ref_point, bearing=90)
            self.lon_thresh = abs(point_east.longitude - ref_point.longitude)

    def connect(self, ip_address='tcp:127.0.0.1:5762'):
        """Permet une connection facile au drone, et l'atente de hearbeat pour s'assurer d'une communcation vivante.

        Args:
            ip_address (str, optional): Adresse ip de connection. 
                Simulation_mavproxy : 'tcp:127.0.0.1:5762' .
                Vrai_connection : 'udp:<ip_ubuntu>:14551' (S'assurer de bien avoir tranmsis le signal de l'antenne sur ce port et alloué la communication udp windows-ubuntu).

        Returns:
            None
        """
        # Create the self.connection
        # Establish connection to MAVLink
        self.connection = mavutil.mavlink_connection(ip_address, baud = 921600)
        print('Waiting for heartbeat...')
        self.connection.wait_heartbeat()
        print("Heartbeat received!")
    
    def global_target(self, wp, acceptance_radius=8e-6, while_moving=None, wait_to_reach=True):
            """Sends a movement command to the drone for a specific global GPS coordinate.

            Args:
                wp (tuple): Target waypoint as (latitude, longitude, altitude in meters).
                acceptance_radius (float, optional): Distance at which the target is considered reached. Defaults to 5 meters.
                while_moving (function, optional): Function to execute while the drone is in transit.
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

            if wait_to_reach:
                # Wait for the waypoint to be reached
                print("Waiting for waypoint to be reached...")
                while not self.is_near_waypoint(self.get_global_pos(), wp, threshold=acceptance_radius, gps=True):
                    if while_moving is not None:
                        while_moving()
                    else:
                        pass
                else:
                    print("Waypoint reached!")

    def is_near_waypoint(self, actual : list, target: list, threshold : float = 2., gps = False):
        """Retoune True si la distance entre le drone et le target est < threshold. Else False.

        Args:
            actual (list): Position actuelle du drone (Coodonnées locales NED : [N, E, -Z])
            target (list): Position visée à comparer (Coodonnées locales NED : [N, E, -Z])
            threshold (float, optional): Distance à parti de laquelle retourner True. Defaults to 2.

        Returns:
            bool: Vrai si le donne est assez proche, False otherwise
        """
        
        if gps:
            return (abs(actual[0] - target[0]) <= self.lat_thresh) and (abs(actual[1] - target[1]) <= self.lon_thresh)
        else:
            return np.linalg.norm(np.array(actual) - np.array(target)) < threshold


    def get_local_pos(self, frequency_hz=60):
        """Permet d'avoir la position locale, et fais une requête pour avoir les données à la fréquence désirée.

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            frequency_hz (int, optional): Fréquence de demandes des données. Defaults to 60.

        Returns:
            Position (list): Position en système de Coodonnées locales NED : [N, E, -Z]) 
        """


        self.message_request(message_type=mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, freq_hz=frequency_hz)

        while self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=False):
            pass  # Discard old messages

        # Loop to receive the most recent message
        while True:
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True) 
            if msg and msg.get_type() == "LOCAL_POSITION_NED":
                print(f"Position: X = {msg.x} m, Y = {msg.y} m, Z = {msg.z} m")
                return [msg.x, msg.y, msg.z]
            # Reduce busy-waiting and ensure responsiveness



    def get_global_pos(self, time_tag=False, heading = False):
        connection = self.connection
        self.message_request(message_type=mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, freq_hz=60)

        while connection.recv_match(type="GLOBAL_POSITION_INT", blocking=False):
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

    def get_rc_value(self, channel):
        """
        Retrieve the raw value of an RC channel.
        
        Args:
            connection: MAVLink connection object.
            channel (int): Channel number to read (1-18).

        Returns:
            int: Raw RC channel value (1000 - 2000).
        """

        # Request the RC_CHANNELS message at 10Hz
        self.message_request(self.connection, mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, freq_hz=60)

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

    def message_request(self, message_type, freq_hz=10):
        """Envoie une requète de message au drone, permet la réception d'un message spécifique, reçu à vitesse spécifique.

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            message_type (id function message): Voir les types de messages de mavlink pouvant être demandé en mode copter
            freq_hz (int, optional): Fréquence voulue d'envoi des données. Defaults to 10 Hz.
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
        """Permet de choisir facilement le mode à partir de sont string

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            mode (str): Identification en lettres du mode
        """
        connection = self.connection
        mode_id = connection.mode_mapping()[mode] #Conversion du mode en son id
        connection.mav.set_mode_send(connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        print(f"Setting mode to {mode}...")


    def arm(self):
        """Arme le drone
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


    def takeoff(self, altitude=10, while_moving = None):
        """Fait décoller le drone. Nécessite le mode 'GUIDED', et que le drone soit armé. 

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            altitude (int, optional): Alitude du drone en m de hauteur par rapport à l'origine. Defaults to 10.
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


    def connect_arm_takeoff(self, ip='tcp:127.0.0.1:5762', height=20):
        """Permet la connection rapide, l'arm du drone et le décollage

        Args:
            ip (str, optional): Voir documentation de connect pour plus d'infos
            height (int, optional): Hauteur de décollage du drone. Defaults to 20.
        """
        self.connect(ip)

        # Set mode to GUIDED
        self.set_mode( "GUIDED")

        self.arm()

        self.takeoff(height)

    def convert_to_global(self, local_delta : tuple, reference_point=None):
        if reference_point is None:
            reference_point = self.home
        """Convertit les coordonnées locales NED en coordonnées GPS globales.

        Args:
            local_pos (list): Position locale en système NED [N, E, -Z].

        Returns:
            tuple: Position GPS globale (latitude, longitude, altitude).
        """
        x, y = local_delta
        point_north = distance(meters=y).destination(reference_point, bearing=0)
        point_final = distance(meters=x).destination(point_north, bearing=90)
        return [point_final.latitude, point_final.longitude]



    def local_target(self, wp, acceptance_radius=5, while_moving = None, turn_into_wp = False):
        """Permet l'envoi facile d'une commande de déplacement du drône aux coordonnées locales en système NED.

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            wp (list): liste des coordonnées en sytème de coordonnées local [N, E, D] (OUI ALTITUDE POSITIVE = NÉGATIF)
            while_moving (fonction) : Chose à faire en attendant l'atteinte du wp
            acceptance_radius (int, optional): Distance à laquelle le drone considère la cible atteinte. Defaults to 5.
        """
        
        connection = self.connection

        yaw_angle = 0
        if turn_into_wp:
            pos = self.get_local_pos
            actual_x, actual_y = pos
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

    def speed_target(self, wp:list, yaw_rate=0):
        yaw_rate = yaw_rate * np.pi / 180  # Convert degrees to radians
        """Permet l'envoi facile d'une commande de vitesse du drône dans le système de référence de celui-ci (En avant, à droite, en bas).

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            wp (list): liste des coordonnées en sytème de [Avant, Droite, Bas] (OUI ALTITUDE POSITIVE = NÉGATIF)
            yaw_rate (float, optional): Vitesse de rotation du drone autour de son axe vertical. Defaults to 0. EN degrés par seconde.
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


    def RTL(self, while_moving = None):
        """Envoie une commande de RTL (return to launch). Attends que le drone soit atteri, une fois atteri, le drone est désarmé et la connection se ferme automatiquement, indiquant la fin de la mission.

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
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



    def insert_coordinates_to_csv(self, file_path, coordinates, desc):
        """
        Inserts coordinates into a CSV file. If the file doesn't exist, it creates one with a header.
        
        Parameters:
            file_path (str): Path to the CSV file.
            coordinates (list of tuples): List of (latitude, longitude) coordinates.
            
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

    def append_description_to_last_line(file_path, description):
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


    def spiral_scan(self, largeur_detection = 10, altitude = 10, rayon_scan = 100, safety_margin = 0, center = None):
        """Permet de générer des points à suivre afin de faire le scan d'une zone circulaire, en effectuant une spirale. Permet ausis de mesurer le temps pris pour faire l'ensemble du scan.

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            largeur_detection (int, optional): Distance horizontale sur laquelle le drone peut détecter un émetter. Defaults to 10.
            altitude (int, optional): Hauteur relative du home à laquelle effecteur le scan. Defaults to 10.
            rayon_scan (int, optional): Rayon de la zone à scanner. Defaults to 100.
            safety_margin (int, optional): Ajout de distance au rayon afin de compenser une erreur de positionnement initial. Defaults to 0.
            center (local_pos, optional): Coordonnées locales du centre du scan. Si laissé à None, prendre la position initiale quand la fonction est appelée.
        """
        if center is None:
            pos = self.get_local_pos()
        else:
            pos = center
        
        espacement = largeur_detection
        nombre_de_tours = rayon_scan / espacement

        rayon_scan += safety_margin

        # Spiral parameters
        theta_spiral = np.linspace(0, 2 * np.pi*nombre_de_tours, 100)
        b = espacement/(2*np.pi)
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


    def rectilinear_scan(self, largeur_detection = 10, altitude = 10, rayon_scan = 100, safety_margin = 0, center = None):
        """Permet de générer des points à suivre afin de faire le scan d'une zone circulaire, en effectuant une forme rectilinéaire. Permet ausis de mesurer le temps pris pour faire l'ensemble du scan.

        Args:
            connection (mavlink connection): Connection au drone, souvent appelée master ou connection
            largeur_detection (int, optional): Distance horizontale sur laquelle le drone peut détecter un émetter. Defaults to 10.
            altitude (int, optional): Hauteur relative du home à laquelle effecteur le scan. Defaults to 10.
            rayon_scan (int, optional): Rayon de la zone à scanner. Defaults to 100.
            safety_margin (int, optional): Ajout de distance au rayon afin de compenser une erreur de positionnement initial. Defaults to 0.
            center (local_pos, optional): Coordonnées locales du centre du scan. Si laissé à None, prendre la position initiale quand la fonction est appelée.
        """
        if center is None:
            pos = self.get_local_pos()
        else:
            pos = center
        
        
        e = largeur_detection
        radius = rayon_scan
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

    def generate_scan_points(scan_width=2, radius_of_scan = 13):
        e = scan_width
        radius = radius_of_scan
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

            return x,y