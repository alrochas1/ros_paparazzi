# File with the info about the messages

# For Telemetry messages
class TelemetryData():
    def __init__(self):
        self.time = 0.0
        self.longitude = 0.0
        self.latitude = 0.0
        self.altitude = 0.0
        self.wp_id = 0
        self.callbacks = []

    def update(self, time, longitude, latitude, altitude, wp_id):
        self.time = time
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude
        self.wp_id = wp_id
        self.send_callback()

    def send_callback(self):
        for callback in self.callbacks:
            callback(self)

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def recover(self):
        return [self.time, self.longitude, self.latitude, self.altitude, self.wp_id]

    def __repr__(self):
        return f"TelemetryData[{self.wp_id}](time= {self.time}, longitude={self.longitude}, latitude={self.latitude}, altitude={self.altitude})"
    

# For Datalink messages
class WaypointData():
    def __init__(self):
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.wp_id = 0

    def update(self, lat, lon, alt, id):
        self.lat = int(lat)
        self.lon = int(lon)
        self.alt = int(alt)
        self.wp_id = int(id)

    def recover(self):
        return [self.lat, self.lon, self.alt, self.wp_id]

    def __repr__(self):
        return f"WaypointData(Latitud={self.lat}, Longitud={self.lon}, Altitud={self.alt}, ID = {self.wp_id})"
    

class IMU_Data():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.time = 0.0
        self.callbacks = []

    def update(self, time, x, y, z):
        self.x = int(x)
        self.y = int(y)
        self.z = int(z)
        self.time = time
        self.send_callback()

    def send_callback(self):
        for callback in self.callbacks:
            callback(self)

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def recover(self):
        return [self.x, self.y, self.z]

    def __repr__(self):
        return f"IMU Data(x={self.x}, y={self.y}, z={self.z})"
    

class GPS_Data():
    def __init__(self):
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.callbacks = []

    def update(self, lat, lon, alt):
        self.lat = int(lat)
        self.lon = int(lon)
        self.alt = int(alt)
        self.send_callback()

    def send_callback(self):
        for callback in self.callbacks:
            callback(self)

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def recover(self):
        return [self.lat, self.lon, self.alt]

    def __repr__(self):
        return f"WaypointData(Latitud={self.lat}, Longitud={self.lon}, Altitud={self.alt}"
    

class Lidar_Data():
    def __init__(self):
        self.distance = 0
        self.callbacks = []

    def update(self, dist):
        self.distance = float(dist)
        self.send_callback()

    def recover(self):
        return [self.distance]
    
    def send_callback(self):
        for callback in self.callbacks:
            callback(self)

    def register_callback(self, callback):
        self.callbacks.append(callback)

    def __repr__(self):
        return f"LidarData = {self.distance} m"

    

# Initialize the messages variables
tiempo = 0
telemetry_data = TelemetryData()    # For the telemetry GPS message
waypoint_data = WaypointData()      # For sending Waypoints to the AP
home_data = TelemetryData()         # For sending the HOME position to the computer
imu_data = IMU_Data()
gps_data = GPS_Data()
lidar_data = Lidar_Data()

