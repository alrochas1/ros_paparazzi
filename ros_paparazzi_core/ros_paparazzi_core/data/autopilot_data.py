# File with the info about the messages

# For Telemetry messages
class TelemetryData():
    def __init__(self):
        self.time = 0.0
        self.longitude = 0.0
        self.latitude = 0.0
        self.altitude = 0.0

    def update(self, time, longitude, latitude, altitude):
        self.time = time
        self.longitude = longitude
        self.latitude = latitude
        self.altitude = altitude

    def recover(self):
        return [self.time, self.longitude, self.latitude, self.altitude]

    def __repr__(self):
        return f"TelemetryData(time= {self.time}, longitude={self.longitude}, latitude={self.latitude}, altitude={self.altitude})"
    

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

    

# Initialize the messages variables
tiempo = 0
telemetry_data = TelemetryData()
waypoint_data = WaypointData()





    


