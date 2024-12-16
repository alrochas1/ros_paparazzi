import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix

from ros_paparazzi_core.simulator import sim_functions
from ros_paparazzi_core.aux import geo_tools
from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_interfaces.msg import KalmanUpdate

import os
import time

class SIM_GPS(Node):

    def __init__(self):
        super().__init__('SIM_GPS')
        self.GPS_publisher = self.create_publisher(NavSatFix, 'sensors/gps', 10)
        self.KalmanPublisher = self.create_publisher(KalmanUpdate, 'kalman/update', 10)

        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        filepath_gps = os.path.join(base_dir, "gps_data.txt")
        filepath_attitude = os.path.join(base_dir, "attitude_data.txt")

        # Lee los datos del GPS
        sim_functions.read_txt(self, filepath_gps)
        self.t = sim_functions.get_column(self.data, 0)
        self.lat = sim_functions.get_column(self.data, 5)
        self.lon = sim_functions.get_column(self.data, 6)
        self.vx = sim_functions.get_column(self.data, 9)
        self.vy = sim_functions.get_column(self.data, 10)
        self.vz = sim_functions.get_column(self.data, 11)

        # Lee los datos de actitud
        sim_functions.read_txt(self, filepath_attitude)
        self.t2 = sim_functions.get_column(self.data, 0)
        self.theta = sim_functions.get_column(self.data, 3) # rad

        self.time_index = 0


    def run(self):

        while rclpy.ok():
            if self.time_index >= len(self.t):
                self.get_logger().info("Rebooting GPS Simulator")
                self.time_index = 0

            # Rellena los mensajes de ambos topics
            kf_msg = KalmanUpdate()
            msg = NavSatFix()
            kf_msg.gps_speed.x = float(self.vx[self.time_index] * 1e-02)
            kf_msg.gps_speed.x = float(self.vy[self.time_index] * 1e-02)
            kf_msg.gps_speed.z = float(self.vz[self.time_index] * 1e-02)
            # self.get_logger().info(f'Publishing GPS_Speed [t={self.t[self.time_index]}]: [{msg.x}, {msg.y}]')
            
            msg.latitude = float(self.lat[self.time_index] * 1e-07)
            msg.longitude = float(self.lon[self.time_index] * 1e-07)
            kf_msg.latitude = msg.latitude
            kf_msg.longitude = msg.longitude

            if self.time_index < len(self.t2) - 1:
                kf_msg.theta = float(self.theta[self.time_index])
                self.get_logger().info(f'Publishing Theta [t={self.t2[self.time_index]}]: [{kf_msg.theta} rad]')

            self.GPS_publisher.publish(msg)
            self.KalmanPublisher.publish(kf_msg)
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