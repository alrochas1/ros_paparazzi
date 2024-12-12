import rclpy
from rclpy.node import Node

from ros_paparazzi_interfaces.msg import Waypoint
from sensor_msgs.msg import NavSatFix

from ros_paparazzi_core.simulator import sim_functions

import os

class SIM_GPS(Node):

    def __init__(self):
        super().__init__('GPS_IMU')
        self.GPS_publisher = self.create_publisher(NavSatFix, 'sensors/gps', 10)

        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        filepath = os.path.join(base_dir, "gps_data.txt")
        sim_functions.read_txt(self, filepath)
        
        self.t = sim_functions.get_column(self.data, 1)
        self.lat = sim_functions.get_column(self.data, 5)
        self.lon = sim_functions.get_column(self.data, 6)

        self.time = 0
        self.timer = self.create_timer(1, self.publish_IMU)


    def publish_IMU(self):

        msg = NavSatFix()
        msg.latitude = float(self.lat[self.time]*1e-07)
        msg.longitude = float(self.lon[self.time]*1e-07)
        
        self.GPS_publisher.publish(msg)
        self.get_logger().info(f'Publishing GPS_Data: [{msg.latitude}, {msg.longitude}]')
        self.time += 1
        if self.time >= len(self.t):
            self.time = 0


def main(args=None):

    print("Main")

    rclpy.init(args=args)

    node = SIM_GPS()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()