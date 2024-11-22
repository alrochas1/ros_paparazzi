# This node send the waypoint in the txt to the raspyimport rclpy
# TODO: Delete the txt and use something more elaborated (the Graphic API ??)
# TODO: Maybe made the number of waypoints to send dynamic

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ros_paparazzi_interfaces.msg import Waypoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from pyproj import Proj, Transformer    # Library for the coordenates calc

# Probably not needed
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

import os # QUITAR (en un futuro)

# For convert the (x,y) to (lat, lon)
def ltp_to_wgs84(origin_lat, origin_lon, x, y):
    source_proj = f"+proj=ortho +lat_0={origin_lat} +lon_0={origin_lon}"
    transformer = Transformer.from_proj(Proj(source_proj), Proj("EPSG:4326"), always_xy=True)

    lon, lat = transformer.transform(x, y)
    return lat, lon


class Waypoint_Sender(Node):

    def __init__(self):
        super().__init__('Waypoint_Sender')
        self.publisher = self.create_publisher(Waypoint, 'datalink_gps', qos_profile)

        self.declare_parameter('units', 'LTP')
        self.units = self.get_parameter('units').get_parameter_value().string_value

        # The node will send the waypoint just one time
        time.sleep(0.5)
        self.wait_subscribers()
        self.send_waypoint()
        self.get_logger().info("All the waypoints were sent. Shutting down")


    def wait_subscribers(self, timeout=5):
        elapsed_time = 0
        while self.publisher.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscribers...')
            time.sleep(0.5)
            elapsed_time += 0.5
            if elapsed_time >= timeout:
                self.get_logger().warning('No subscribers connected after waiting.')
                break


    def send_waypoint(self):

        # Temporal
        [lat, lon, alt, wp_id] = self.get_data()

        msg = Waypoint()
        # msg.header.stamp.sec = int(autopilot_data.tiempo)
        # msg.header.stamp.nanosec = int(1e+9*(autopilot_data.tiempo - int(autopilot_data.tiempo)))
        msg.position.longitude = float(lon)
        msg.position.latitude = float(lat)
        msg.position.altitude = float(alt)      # Manda una altitud fija
        msg.wp_id = int(wp_id)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing data: [{lat * 1e-7:.7f}, {lon * 1e-7:.7f}] --> WP_ID = [{wp_id}]')


    # TEMPORAL: To simulate the data ------------------------------------------------------
    def get_data(self):
        try:

            # Esto es una cutrez, pero me vale para probar
            current_file_dir = os.path.dirname(os.path.abspath(__file__))
            workspace_dir = os.path.abspath(os.path.join(current_file_dir, "../../../../../.."))
            if self.units == 'WGS84':
                config_file_path = os.path.join(workspace_dir, "src", "ros_paparazzi", "data_WGS.txt")
            else:
                config_file_path = os.path.join(workspace_dir, "src", "ros_paparazzi", "data_LTP.txt")
                origin_lat = 40.4509250
                origin_lon = -3.7271889

            with open(config_file_path, "r") as file:
                lines = file.readlines()
                
                # For moving randomly
                # latitude = int(40.450925 * 1e7 + (random.randint(-10000, 10000)))
                # longitude = int(-3.727189 * 1e7 + (random.randint(-10000, 10000)))

                # For using the txt
                if self.units == 'WGS84':
                    latitude = int(self.get_value_from_line(lines, "latitude"))
                    longitude = int(self.get_value_from_line(lines, "longitude"))
                else:
                    x = self.get_value_from_line(lines, "x")
                    y = self.get_value_from_line(lines, "y")
                    latitude, longitude = ltp_to_wgs84(origin_lat, origin_lon, x, y)
                    latitude = latitude * 1e7
                    longitude = longitude * 1e7
            
                # altitude = int(self.get_value_from_line(lines, "altitude"))
                altitude = 650*1e+03
                wp_id = int(self.get_value_from_line(lines, "waypoint_id"))
                
                return latitude, longitude, altitude, wp_id
            
        except Exception as e:
            self.get_logger().error(f"Error al leer el archivo: {e}")
            return 404506399, -37260463, 650000, 14  # Valores por defecto en caso de error
        
    def get_value_from_line(self, lines, key):
        for line in lines:
            if line.startswith(key):
                return line.split("=")[1].strip()
        return None
    
    # ------------------------------------------------------------------------------------------


def main(args=None):

    rclpy.init(args=args)
    waypoint_sender = Waypoint_Sender()
    waypoint_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
