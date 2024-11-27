import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from ros_paparazzi_core.data import gcs_data

# telemetry_data = {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0}

class Telemetry_Receiver(Node):
    def __init__(self):
        super().__init__('Telemetry_Receiver')
        self.subscription = self.create_subscription(
            NavSatFix,
            'telemetry_gps',  # Nombre del topic
            self.telemetry_callback,
            10  # Profundidad del buffer
        )

    def telemetry_callback(self, msg):
        gcs_data.telemetry_data.update(msg.latitude, msg.longitude, msg.altitude)
        # self.get_logger().info(gcs_data.telemetry_data)



