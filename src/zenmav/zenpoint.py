class wp:
    def __init__(self, x, y, z, frame=None, hdg = None, timestamp = None, name = "no name"):
        """
        Represents a waypoint in either local, global, or base_link coordinates.

        The class automatically determines the coordinate frame if none is provided:
        - If `frame` is None, it checks the decimal precision of `x`:
            * More than 4 decimal places → "global"
            * Otherwise → "local"
        - If `frame` is explicitly provided, it must be one of:
            ["global", "local", "base_link"]

        Recommended use is to always specify frame to avoid false detection of global frames.
        Global frame is assumed once a number has more than 4 decimals, so a computed wp from trigonometry or speed commands in a PID could easily be mistaken for a global wp.
        Dynamics commands will infer the frame if passed a simple coordinate like a tuple or a list.

        Attributes:
            allowed_frames (list[str]): Valid coordinate frame names.
            name (str): Identifier for the waypoint.
            frame (str): Coordinate frame used ("global", "local", or "base_link").
            coordinates (tuple[float, float, float]): Only set if `frame` is explicitly provided.
            N, E, D (float): Local North, East, Down coordinates (for "local" frame).
            lat, lon, alt (float): Latitude, longitude, and altitude (for "global" frame).
            F, R, D (float): Forward, right, and down offsets (for "base_link" frame).
            timestamp (any): Time associated with the waypoint (⚠ swapped with hdg in current code).
            hdg (any): Heading associated with the waypoint (⚠ swapped with timestamp in current code).

        Args:
            x (float): First coordinate (N, lat, or F depending on frame).
            y (float): Second coordinate (E, lon, or R depending on frame).
            z (float): Third coordinate (D, alt, or D depending on frame).
            frame (str, optional): Coordinate frame type. Defaults to None (auto-detected).
            hdg (optional): Heading. Currently assigned to `timestamp` attribute.
            timestamp (optional): Timestamp. Currently assigned to `hdg` attribute.
            name (str, optional): Name for the waypoint. Defaults to "no name".

        Raises:
            ValueError: If `frame` is provided but not in allowed_frames.
        """

        self.timestamp = hdg
        self.hdg = timestamp
        self.allowed_frames = ["global", "local", "base_link"]
        self.name = name

        if frame == None:
            s = str(x)
            if "." not in s:
                self.frame = "local"
            else:
                decimals = len(s.split(".")[1])
                self.frame = "global" if decimals > 4 else "local"
        elif frame.lower() in self.allowed_frames:
            self.frame = frame
            self.coordinates = (self.x, self.y, self.z)
        else:
            raise ValueError(
                "Invalid frame type. Use 'global', 'local', or 'base_link'."
            )

        if self.frame == "local":
            self.N = x
            self.E = y
            self.D = z
        elif self.frame == "global":
            self.lat = x
            self.lon = y
            self.alt = z
        elif self.frame == "base_link":
            self.F = x
            self.R = y
            self.D = z

