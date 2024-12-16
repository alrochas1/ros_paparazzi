import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

from ros_paparazzi_core.simulator import sim_functions
from ros_paparazzi_interfaces.msg import KalmanPredict 

import os
import time

class SIM_IMU(Node):

    def __init__(self):
        super().__init__('SIM_IMU')
        self.IMU_publisher = self.create_publisher(Vector3, 'sensors/imu', 10)
        self.KalmanPublisher = self.create_publisher(KalmanPredict, 'kalman/predict', 10)

        # TEMPORALMENTE lo apaño aqui con dos archivos
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        filepath = os.path.join(base_dir, "imu_data.txt")
        sim_functions.read_txt(self, filepath)
        
        self.t = sim_functions.get_column(self.data, 0)
        self.ax = sim_functions.get_column(self.data, 8)
        self.ay = sim_functions.get_column(self.data, 9)
        self.az = sim_functions.get_column(self.data, 10)

        self.time_index = 0


    def run(self):

        while rclpy.ok():
            if self.time_index >= len(self.t):
                self.get_logger().info("Rebooting IMU Simulator")
                self.time_index = 0

            msg = Vector3()
            msg.x = float(self.ax[self.time_index])
            msg.y = float(self.ay[self.time_index])
            msg.z = float(self.az[self.time_index])
            self.IMU_publisher.publish(msg)
            kf = KalmanPredict()
            kf.imu.x = msg.x; kf.imu.y = msg.y; kf.imu.z = msg.z
            self.KalmanPublisher.publish(kf)
            self.get_logger().info(f'Publishing IMU_Data [t={self.t[self.time_index]}]: [{msg.x}, {msg.y}, {msg.z}]')
        
            if self.time_index < len(self.t) - 1:
                sleep_duration = self.t[self.time_index + 1] - self.t[self.time_index]
                sleep_duration = max(0.0, sleep_duration)   # Por si acaso
                time.sleep(sleep_duration)

            self.time_index += 1



def main(args=None):
    rclpy.init(args=args)

    node = SIM_IMU()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()