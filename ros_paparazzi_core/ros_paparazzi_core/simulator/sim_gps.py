import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix

from ros_paparazzi_core.simulator import sim_functions
from ros_paparazzi_core.aux import geo_tools
from ros_paparazzi_core.data import gcs_data

import os
import time

class SIM_GPS(Node):

    def __init__(self):
        super().__init__('SIM_GPS')
        self.GPS_publisher = self.create_publisher(NavSatFix, 'sensors/gps', 10)
        self.GPS_Speed_Publisher = self.create_publisher(Vector3, 'sensors/gps/speed', 10)

        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        filepath = os.path.join(base_dir, "gps_data.txt")
        sim_functions.read_txt(self, filepath)
        
        self.t = sim_functions.get_column(self.data, 0)
        self.lat = sim_functions.get_column(self.data, 5)
        self.lon = sim_functions.get_column(self.data, 6)
        self.vx = sim_functions.get_column(self.data, 9)
        self.vy = sim_functions.get_column(self.data, 10)
        self.vz = sim_functions.get_column(self.data, 11)


        self.time_index = 0


    def run(self):

        while rclpy.ok():
            if self.time_index >= len(self.t):
                self.get_logger().info("Rebooting GPS Simulator")
                self.time_index = 0

            msg = Vector3()
            msg.x = float(self.vx[self.time_index] * 1e-02)
            msg.y = float(self.vy[self.time_index] * 1e-02)
            msg.z = float(self.vz[self.time_index] * 1e-02)
            self.GPS_Speed_Publisher.publish(msg)
            self.get_logger().info(f'Publishing GPS_Speed [t={self.t[self.time_index]}]: [{msg.x}, {msg.y}]')

            msg = NavSatFix()
            msg.latitude = float(self.lat[self.time_index] * 1e-07)
            msg.longitude = float(self.lon[self.time_index] * 1e-07)
            self.GPS_publisher.publish(msg)
            self.get_logger().info(f'Publishing GPS_Data [t={self.t[self.time_index]}]: [{msg.latitude}, {msg.longitude}]')


            if self.time_index < len(self.t) - 1:
                sleep_duration = self.t[self.time_index + 1] - self.t[self.time_index]
                sleep_duration = max(0.0, sleep_duration)   # Por si acaso
                time.sleep(sleep_duration)

            self.time_index += 1


def main(args=None):
    rclpy.init(args=args)

    node = SIM_GPS()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()