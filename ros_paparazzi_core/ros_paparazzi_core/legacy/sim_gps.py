import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from ros_paparazzi_core.aux import sim_functions
from ros_paparazzi_interfaces.msg import KalmanUpdate

import os
import time

class SIM_GPS(Node):

    def __init__(self):
        super().__init__('SIM_GPS')
        self.GPS_publisher = self.create_publisher(NavSatFix, 'sensors/gps', 10)
        self.KalmanPublisher = self.create_publisher(KalmanUpdate, 'kalman/update', 10)
        self.SIMCORE_suscriber = self.create_subscription(Header, 'sim/sensor_event', self.sim_callback, 10)

        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        filepath_gps = os.path.join(base_dir, "gps_data.txt")
        filepath_attitude = os.path.join(base_dir, "attitude_data.txt")

        # Lee los datos del GPS
        sim_functions.read_txt(self, filepath_gps)
        self.t_gps = sim_functions.get_column(self.data, 0)
        self.lat = sim_functions.get_column(self.data, 5)
        self.lon = sim_functions.get_column(self.data, 6)
        self.vx = sim_functions.get_column(self.data, 9)
        self.vy = sim_functions.get_column(self.data, 10)
        self.vz = sim_functions.get_column(self.data, 11)

        # Lee los datos de actitud
        sim_functions.read_txt(self, filepath_attitude)
        self.t_attitude = sim_functions.get_column(self.data, 0)
        self.theta_data = sim_functions.get_column(self.data, 3) # rad

        self.gps_index = 0
        self.attitude_index = 0
        self.theta = 0


    def sim_callback(self, msg):

        if msg.frame_id == 'gps':

            if self.gps_index >= len(self.t_gps):
                self.get_logger().info("Rebooting GPS Simulator")
                self.gps_index = 0

            # Rellena los mensajes de ambos topics
            kf_msg = KalmanUpdate()
            msg = NavSatFix()
            kf_msg.gps_speed.x = float(self.vx[self.gps_index] * 1e-02)
            kf_msg.gps_speed.x = float(self.vy[self.gps_index] * 1e-02)
            kf_msg.gps_speed.z = float(self.vz[self.gps_index] * 1e-02)
            kf_msg.theta = float(self.theta)
            
            msg.latitude = float(self.lat[self.gps_index] * 1e-07)
            msg.longitude = float(self.lon[self.gps_index] * 1e-07)
            kf_msg.latitude = msg.latitude
            kf_msg.longitude = msg.longitude

            self.GPS_publisher.publish(msg)
            self.KalmanPublisher.publish(kf_msg)

            self.get_logger().info(f'Publishing GPS_Speed [t={self.t_gps[self.gps_index]}]: [{msg.latitude}, {msg.longitude}]')
            self.gps_index += 1


        elif msg.frame_id == 'attitude':
            
            if self.attitude_index >= len(self.t_attitude):
                self.get_logger().info("Rebooting Attitude Simulator")
                self.attitude_index = 0

            self.theta = float(self.theta_data[self.attitude_index])
            self.get_logger().info(f'Publishing Theta [t={self.t_attitude[self.attitude_index]}]: [{self.theta} rad]')
            self.attitude_index += 1



def main(args=None):
    rclpy.init(args=args)

    node = SIM_GPS()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()