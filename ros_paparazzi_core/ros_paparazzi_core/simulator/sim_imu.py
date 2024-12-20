import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

from ros_paparazzi_core.aux import sim_functions
from ros_paparazzi_interfaces.msg import KalmanPredict 

import os
import time

class SIM_IMU(Node):

    def __init__(self):
        super().__init__('SIM_IMU')
        self.IMU_publisher = self.create_publisher(Vector3, 'sensors/imu', 10)
        self.KalmanPublisher = self.create_publisher(KalmanPredict, 'kalman/predict', 10)
        self.SIMCORE_suscriber = self.create_subscription(Header, 'sim/sensor_event', self.sim_callback, 10)

        # TEMPORALMENTE lo apaño aqui con dos archivos
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        filepath = os.path.join(base_dir, "imu_data.txt")
        sim_functions.read_txt(self, filepath)
        
        self.t_imu = sim_functions.get_column(self.data, 0)
        self.ax = sim_functions.get_column(self.data, 8)
        self.ay = sim_functions.get_column(self.data, 9)
        self.az = sim_functions.get_column(self.data, 10)

        self.imu_index = 0


    def sim_callback(self, msg):

        if msg.frame_id == 'imu':

            if self.imu_index >= len(self.t_imu):
                self.get_logger().info("Rebooting IMU Simulator")
                self.imu_index = 0

            msg = Vector3()
            msg.x = float(self.ax[self.imu_index])
            msg.y = float(self.ay[self.imu_index])
            msg.z = float(self.az[self.imu_index])
            self.IMU_publisher.publish(msg)
            kf = KalmanPredict()
            kf.imu.x = msg.x; kf.imu.y = msg.y; kf.imu.z = msg.z
            
            self.get_logger().info(f'Publishing IMU_Data [t={self.t_imu[self.imu_index]}]: [{msg.x}, {msg.y}, {msg.z}]')
            self.imu_index += 1


def main(args=None):
    rclpy.init(args=args)

    node = SIM_IMU()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()