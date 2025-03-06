import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from ros_paparazzi_interfaces.msg import Waypoint

from ros_paparazzi_core.aux import geo_tools



class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('Simple_SLAM')

        self.scan_sub = self.create_subscription(LaserScan, 'sensors/lidar', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Waypoint, 'waypoints/telemetry_gps', self.pos_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.ref_sub = self.create_subscription(Waypoint, '/waypoints/reference', self.ref_callback, 10)

        self.robot_x = 0; self.robot_y = 0; self.robot_yaw = 0
        self.lat0 = 0; self.lon0 = 0

        # Parámetros del mapa
        self.map_resolution = 0.05  # 5 cm por celda
        self.map_width = 500        
        self.map_height = 500    
        self.map_origin_x = -2.5    # Origen en X (en metros)
        self.map_origin_y = -2.5    # Origen en Y (en metros)

        # Inicializar el mapa
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)




    def ref_callback(self, msg):
        self.lat0 = msg.gps.latitude
        self.lon0 = msg.gps.longitude


    def pos_callback(self, msg):
        lat = msg.gps.latitude
        lon = msg.gps.longitude

        # TEST
        lat = 40.4506486
        lon = -3.7271496
        self.robot_x, self.robot_y = geo_tools.wgs84_to_ltp(self.lat0, self.lon0, lat, lon)
        print(f"Punto de Origen = [{self.lat0}, {self.lon0}]")
        print(f"Posicion del rover = [{self.robot_x}, {self.robot_y}]")

        # Calcular el yaw (orientación)
        orientation = msg.gps.altitude # Es el yaw, reutilizo el mensaje
        self.robot_yaw = 2*np.pi - orientation


    def scan_callback(self, msg):
        # Procesar datos del LiDAR (solo si ya esta posicionado)
        if self.lat0 != 0 or self.lon0 != 0:
            for i, distance in enumerate(msg.ranges):
                if msg.range_min <= distance <= msg.range_max:
                    # angle = msg.angle_min + i * msg.angle_increment
                    angle = -msg.angle_increment*np.pi/180
                    print(f"Distancia = {distance}, Angle = {180*(self.robot_yaw - angle)/np.pi}")
                    # TODO: REVISAR. Creo que esta mal (a lo mejor no)
                    obstacle_x = self.robot_x + distance * np.cos(self.robot_yaw - angle)
                    obstacle_y = self.robot_y + distance * np.sin(self.robot_yaw - angle)
                    print(f"Obstaculo en [{obstacle_x}, {obstacle_y}]")

                    # Convertir la posición del obstáculo a coordenadas del mapa
                    map_x = int((obstacle_x - self.map_origin_x) / self.map_resolution)
                    map_y = int((obstacle_y - self.map_origin_y) / self.map_resolution)
                    map_x0 = int((self.robot_x - self.map_origin_x) / self.map_resolution)
                    map_y0 = int((self.robot_y - self.map_origin_y) / self.map_resolution)

                    for x, y in self.bresenham(map_x0, map_y0, map_x, map_y):
                        if 0 <= x < self.map_width and 0 <= y < self.map_height:
                            self.map_data[y, x] = 0  # Celda libre

                    # Actualizar el mapa
                    if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                        self.map_data[map_y, map_x] = 100
                        # self.update_map(map_x, map_y, True)
            
            
            self.publish_map()


    # def update_map(self, map_x, map_y, occupied):
    #     prior = self.map_data[map_y, map_x]  # Valor actual en la celda
        
    #     if occupied:
    #         self.map_data[map_y, map_x] = min(prior + 10, 100)
    #     else:
    #         self.map_data[map_y, map_x] = max(prior - 5, 0)



    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        # Configurar la información del mapa
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y

        # Convertir la matriz del mapa a un array unidimensional
        map_msg.data = self.map_data.flatten().tolist()

        self.map_pub.publish(map_msg)
        # self.get_logger().info('Mapa publicado')


    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points



def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


