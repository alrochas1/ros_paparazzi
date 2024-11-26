# This is the old node that was used to send and receive the data from the raspy
# Not used

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ros_paparazzi_interfaces.msg import Waypoint

from pyproj import Proj, Transformer    # Library for the coordenates calc

import os # QUITAR (en un futuro)
import time

# For convert the (x,y) to (lat, lon)
def ltp_to_wgs84(origin_lat, origin_lon, x, y):
    source_proj = f"+proj=ortho +lat_0={origin_lat} +lon_0={origin_lon}"
    transformer = Transformer.from_proj(Proj(source_proj), Proj("EPSG:4326"), always_xy=True)

    lon, lat = transformer.transform(x, y)
    return lat, lon


class Computer_Subscriber(Node):

    def __init__(self):
        super().__init__('Computer_Suscriber')
        self.subscription = self.create_subscription(Waypoint, 'telemetry_gps', self.telemetry_callback, 10)
        self.publisher = self.create_publisher(Waypoint, 'datalink_gps', 10)
        # self.create_timer(3, self.datalink_callback)
        self.timer = None

        self.declare_parameter('units', 'LTP')
        self.units = self.get_parameter('units').get_parameter_value().string_value

        # For HOME position (default)
        self.origin_lat = None
        self.update_HOME()
        # self.origin_lon = None

    
    # To send the HOME request message
    def update_HOME(self):

        wp_id = 0
        data = [0.0, 0.0, 0.0, wp_id]
        self.send_datalink(data)    # Here the lat, lon, alt is not necesary
        

    # To send the waypoints to the AP
    def datalink_callback(self):

        # Temporal
        # [lat, lon, alt, wp_id] = self.get_data()
        data = self.get_data()
        self.send_datalink(data)

            
    # To publish in the datalink topic
    def send_datalink(self, data):

        [lat, lon, alt, wp_id] = data
        
        msg = Waypoint()
        # msg.header.stamp.sec = int(autopilot_data.tiempo)
        # msg.header.stamp.nanosec = int(1e+9*(autopilot_data.tiempo - int(autopilot_data.tiempo)))
        msg.position.longitude = float(lon)
        msg.position.latitude = float(lat)
        msg.position.altitude = float(alt)      # Manda una altitud fija
        msg.wp_id = int(wp_id)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing data: [{lat * 1e-7:.7f}, {lon * 1e-7:.7f}] --> WP_ID = [{wp_id}]')


    # To receive the messages from the AP
    def telemetry_callback(self, msg):

        if msg.wp_id == 0:  # If id = 0, its the GPS Telemetry
            self.get_logger().info(f'Receiving data: [{msg.gps.latitude:.7f}, {msg.gps.longitude:.7f}, {msg.gps.altitude:.2f}]')
        
        elif msg.wp_id == 1:   # If id = 1, its the home coordinate
            self.origin_lat = msg.gps.latitude
            self.origin_lon = msg.gps.longitude
            self.get_logger().info(f'Receiving HOME: [{msg.gps.latitude:.7f}, {msg.gps.longitude:.7f}, {msg.gps.altitude:.2f}]')
            if self.timer == None:
                self.timer = self.create_timer(3, self.datalink_callback)    
            
        else:
            self.get_logger().info(f'ERROR. wp_id {msg.wp_id} not supported')


    # TEMPORAL: To simulate the data ------------------------------------------------------
    def get_data(self):
        try:

            # Esto es un poco cutre, pero me vale para probar
            current_file_dir = os.path.dirname(os.path.abspath(__file__))
            workspace_dir = os.path.abspath(os.path.join(current_file_dir, "../../../../../.."))
            if self.units == 'WGS84':
                config_file_path = os.path.join(workspace_dir, "src", "ros_paparazzi", "data_WGS.txt")
            else:
                config_file_path = os.path.join(workspace_dir, "src", "ros_paparazzi", "data_LTP.txt") 
                

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
                    latitude, longitude = ltp_to_wgs84(self.origin_lat, self.origin_lon, x, y)
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

    computer_subscriber = Computer_Subscriber()

    rclpy.spin(computer_subscriber)

    computer_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

