import rclpy
from rclpy.node import Node

from ros_paparazzi_interfaces.msg import Waypoint
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix

from ros_paparazzi_core.simulator import sim_functions

import os

class SIM_IMU(Node):

    def __init__(self):
        super().__init__('SIM_IMU')
        self.IMU_publisher = self.create_publisher(Vector3, 'sensors/imu', 10)

        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        filepath = os.path.join(base_dir, "imu_data.txt")
        sim_functions.read_txt(self, filepath)
        
        self.t = sim_functions.get_column(self.data, 1)
        self.ax = sim_functions.get_column(self.data, 8)
        self.ay = sim_functions.get_column(self.data, 9)
        self.az = sim_functions.get_column(self.data, 10)

        self.time = 0
        self.timer = self.create_timer(1, self.publish_IMU)


    def publish_IMU(self):

        msg = Vector3()
        msg.x = float(self.ax[self.time])
        msg.y = float(self.ay[self.time])
        msg.z = float(self.az[self.time])

        self.IMU_publisher.publish(msg)
        self.get_logger().info(f'Publishing IMU_Data: [{msg.x}, {msg.y}, {msg.z}]')
        self.time += 1
        if self.time >= len(self.t):
            self.time = 0


def main(args=None):

    print("Main")

    rclpy.init(args=args)

    node = SIM_IMU()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()