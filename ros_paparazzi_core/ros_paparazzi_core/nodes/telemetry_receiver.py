# TODO: Change the topics to waypoint

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ros_paparazzi_interfaces.msg import Waypoint

from ros_paparazzi_core.data import gcs_data

# telemetry_data = {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0}

class Telemetry_Receiver(Node):
    def __init__(self):
        super().__init__('Telemetry_Receiver')
        self.subscription = self.create_subscription(Waypoint, 'telemetry_gps', self.telemetry_callback, 10)
        self.publisher = self.create_publisher(NavSatFix, 'waypoints/home', 10)

    def telemetry_callback(self, msg):
        if msg.wp_id == 1:
            self.update_home(msg)
        elif msg.wp_id == 0:
            gcs_data.telemetry_data.update(msg.latitude, msg.longitude, msg.altitude)
        

    def update_home(self, gps):

        msg = NavSatFix()
        msg.latitude = gps.latitude
        msg.longitude = gps.longitude
        msg.altitude = gps.altitude

        self.publisher.publish(msg)





