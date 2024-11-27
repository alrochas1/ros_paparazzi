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
    

telemetry_data = TelemetryData()