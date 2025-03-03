import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from ros_paparazzi_interfaces.msg import Waypoint



class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('Simple_SLAM')

        self.scan_sub = self.create_subscription(LaserScan, 'sensors/lidar', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Waypoint, 'waypoints/telemetry_gps', self.pos_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        self.robot_x = 0; self.robot_y = 0; self.robot_yaw = 0

        # Parámetros del mapa
        self.map_resolution = 0.01  # 5 cm por celda
        self.map_width = 500        
        self.map_height = 500    
        self.map_origin_x = -2.5    # Origen en X (en metros)
        self.map_origin_y = -2.5    # Origen en Y (en metros)

        # Inicializar el mapa
        self.map_data = np.zeros((self.map_height, self.map_width), dtype=np.int8)


    def pos_callback(self, msg):
        self.robot_x = msg.gps.latitude*0
        self.robot_y = msg.gps.altitude*0

        # Calcular el yaw (orientación)
        orientation = msg.gps.altitude # Es el yaw, reutilizo el mensaje
        self.robot_yaw = 2*np.pi - orientation


    def scan_callback(self, msg):
        # Procesar datos del LiDAR
        for i, distance in enumerate(msg.ranges):
            if msg.range_min <= distance <= msg.range_max:
                # angle = msg.angle_min + i * msg.angle_increment
                angle = -msg.angle_increment*np.pi/180
                print(f"Distancia = {distance}, Angle = {180*(self.robot_yaw - angle)/np.pi}")
                # TODO: REVISAR. Creo que esta mal
                obstacle_x = self.robot_x + distance * np.cos(self.robot_yaw - angle)
                obstacle_y = self.robot_y + distance * np.sin(self.robot_yaw - angle)
                print(f"Obstaculo en [{obstacle_x}, {obstacle_y}]")

                # Convertir la posición del obstáculo a coordenadas del mapa
                map_x = int((obstacle_x - self.map_origin_x) / self.map_resolution)
                map_y = int((obstacle_y - self.map_origin_y) / self.map_resolution)

                # Actualizar el mapa
                if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                    self.map_data[map_y, map_x] = 100
        
        self.publish_map()



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


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


