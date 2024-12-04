# File with the data to the bokeh

class TelemetryData():
    def __init__(self):
        self.longitude = 0.0
        self.latitude = 0.0
        self.altitude = 0.0

    def update(self, longitude, latitude, altitude):
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude

    def recover(self):
        return [self.longitude, self.latitude, self.altitude]

    def __repr__(self):
        return f"TelemetryData(longitude={self.longitude}, latitude={self.latitude}, altitude={self.altitude})"
    

class WaypointData():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.wp_id = 0

    def update(self, x, y, wp_id):
        if x is not None:
            self.x = float(x)
        if y is not None:
            self.y = float(y)
        if wp_id is not None:
            self.wp_id = wp_id

    def recover(self):
        return [self.x, self.y, self.wp_id]

    def __repr__(self):
        return f"WaypointData(X={self.x}, Y={self.y}, ID = {self.wp_id})"
    


class USV_State():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

    def update_position(self, x, y):
        if x is not None:
            self.x = float(x)
        if y is not None:
            self.y = float(y)

    def recover_position(self):
        return [self.x, self.y]

    def __repr__(self):
        return f"State(X={self.x}, Y={self.y})"
    


class TerminalData():
    def __init__(self):
        self.msg = "<b>Terminal Output:</b><br>"

    def update_terminal(self, msg):
        if msg is not None:
            self.msg = self.msg + msg

    def recover_message(self):
        return self.msg

    

telemetry_data = TelemetryData()
waypoint_data = WaypointData()
usv_state = USV_State()
terminal_data = TerminalData()

imu_data = [0, 0, 0]
gps_data = [0, 0]
time = 0.0

origin = [40.4509250, -3.7271889, 650]
raspy_status = False