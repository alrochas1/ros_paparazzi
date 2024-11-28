# This is a temporal node for reading the data from the txt

import rclpy
import os
from rclpy.node import Node
# from ros_paparazzi_interfaces.msg import Waypoint
from ros_paparazzi_interfaces.srv import GetWaypoint

from pyproj import Proj, Transformer    # Library for the coordenates calc

# For convert the (x,y) to (lat, lon)
def ltp_to_wgs84(origin_lat, origin_lon, x, y):
    source_proj = f"+proj=ortho +lat_0={origin_lat} +lon_0={origin_lon}"
    transformer = Transformer.from_proj(Proj(source_proj), Proj("EPSG:4326"), always_xy=True)

    lon, lat = transformer.transform(x, y)
    return lat, lon


class DataProvider(Node):

    def __init__(self):
        super().__init__('data_provider')
        self.srv = self.create_service(GetWaypoint, 'get_waypoint', self.get_waypoint_callback)

        self.declare_parameter('units', 'LTP')
        self.units = self.get_parameter('units').get_parameter_value().string_value


    # (request is not accesed because there is nothing at the request)
    def get_waypoint_callback(self, request, response):

        # Get the data from the txt
        [latitude, longitude, altitude, wp_id] = self.get_data()
        
        # Asignamos los valores a la respuesta del servicio
        response.latitude = latitude
        response.longitude = longitude
        response.altitude = altitude
        response.wp_id = wp_id

        self.get_logger().info(f'Returned waypoint: lat={latitude}, lon={longitude}, alt={altitude}, id={wp_id}')
        return response


    # Auxiliar function
    def get_value_from_line(self, lines, key):
        for line in lines:
            if line.startswith(key):
                return line.split("=")[1].strip()
        return None
    
    # For reading the txt
    def get_data(self):
        try:
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
                    latitude = float(self.get_value_from_line(lines, "latitude"))
                    longitude = float(self.get_value_from_line(lines, "longitude"))
                else:
                    x = self.get_value_from_line(lines, "x")
                    y = self.get_value_from_line(lines, "y")
                    latitude, longitude = ltp_to_wgs84(origin_lat, origin_lon, x, y)
                    latitude = float(latitude)
                    longitude = float(longitude)
            
                # altitude = int(self.get_value_from_line(lines, "altitude"))
                altitude = float(650*1e+03)
                wp_id = int(self.get_value_from_line(lines, "waypoint_id"))
                
                return latitude, longitude, altitude, wp_id
            
        except Exception as e:
            self.get_logger().error(f"Error reading file: {e}") 
            return 404506399, -37260463, 650000, 14  # Default values 
        


# Main function for starting the node
def main(args=None):
    rclpy.init(args=args)
    data_provider = DataProvider()
    rclpy.spin(data_provider)
    data_provider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


