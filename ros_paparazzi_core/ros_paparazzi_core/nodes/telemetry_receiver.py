from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ros_paparazzi_interfaces.msg import Waypoint
from geometry_msgs.msg import Vector3

from ros_paparazzi_core.data import gcs_data

# telemetry_data = {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0}

class Telemetry_Receiver(Node):
    def __init__(self):
        super().__init__('Telemetry_Receiver')
        self.telemetry_subscription = self.create_subscription(Waypoint, 'waypoints/telemetry_gps', self.telemetry_callback, 10)
        self.IMU_subscription = self.create_subscription(Vector3, 'sensors/imu', self.imu_callback, 10)
        self.GPS_subscription = self.create_subscription(NavSatFix, 'sensors/gps', self.gps_callback, 10)
        self.publisher = self.create_publisher(NavSatFix, 'waypoints/home', 10)

    def telemetry_callback(self, msg):
        if msg.wp_id == 1:
            self.update_home(msg)
        elif msg.wp_id == 0:
            gcs_data.telemetry_data.update(msg.gps.latitude, msg.gps.longitude, msg.gps.altitude)
        

    def imu_callback(self, msg):
        gcs_data.imu_data = [msg.x/1024, msg.y/1024, msg.z/1024]

    def gps_callback(self, msg):
        gcs_data.gps_data = [msg.latitude, msg.longitude]

    def update_home(self, msg):

        home = NavSatFix()
        home.latitude = msg.gps.latitude
        home.longitude = msg.gps.longitude
        home.altitude = msg.gps.altitude

        self.publisher.publish(home)
        self.get_logger().info(f'Home Updated: lat={home.latitude}, lon={home.longitude}')





