# This is the node that will run on the computer

import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from autopilot_interfaces.msg import Waypoint


class Computer_Subscriber(Node):

    def __init__(self):
        super().__init__('Computer_Suscriber')
        self.subscription = self.create_subscription(NavSatFix, 'telemetry_gps', self.telemetry_callback, 10)
        self.publisher = self.create_publisher(Waypoint, 'datalink_gps', 10)
        self.create_timer(3, self.datalink_callback)
        
    def telemetry_callback(self, msg):
        self.get_logger().info(f'Receiving data: [{msg.longitude}, {msg.latitude}, {msg.altitude}]')

    def datalink_callback(self):

        # Temporal
        [lat, lon, alt, wp_id] = self.get_data()

        msg = Waypoint()
        # msg.header.stamp.sec = int(autopilot_data.tiempo)
        # msg.header.stamp.nanosec = int(1e+9*(autopilot_data.tiempo - int(autopilot_data.tiempo)))
        msg.position.longitude = float(lon)
        msg.position.latitude = float(lat)
        msg.position.altitude = float(alt)
        msg.wp_id = wp_id
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing data: [{lat * 1e-7:.7f}, {lon * 1e-7:.7f}]')


    # TEMPORAL: To simulate the data
    def get_data(self):
        try:
            with open("~/data.txt", "r") as file:
                lines = file.readlines()
                
                # For moving randomly
                latitude = int(40.4506399 * 1e7 + (random.randint(-10000, 10000)))
                longitude = int(-3.7260463 * 1e7 + (random.randint(-10000, 10000)))

                # For using the txt
                latitude = int(self.get_value_from_line(lines, "latitude"))
                longitude = int(self.get_value_from_line(lines, "longitude"))
                altitude = int(self.get_value_from_line(lines, "altitude"))
                wp_id = int(self.get_value_from_line(lines, "waypoint_id"))
                
                return latitude, longitude, altitude, wp_id
            
        except Exception as e:
            self.get_logger().error(f"Error al leer el archivo: {e}")
            return 40.4506399, -3.7260463, 650.0, 14  # Valores por defecto en caso de error




def main(args=None):
    rclpy.init(args=args)

    computer_subscriber = Computer_Subscriber()

    rclpy.spin(computer_subscriber)

    computer_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

