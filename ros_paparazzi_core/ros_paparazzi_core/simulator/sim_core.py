import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

from ros_paparazzi_core.aux import sim_functions

import os
import time

class SIM_CORE(Node):

    def __init__(self):
        super().__init__('SIM_CORE')
        self.publisher = self.create_publisher(Header, 'sim/sensor_event', 10)

        # Configuración
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        base_dir = sim_functions.load_config(self, config_path)
        
        gps_time = sim_functions.get_time_vector(os.path.join(base_dir, "gps_data.txt"))
        imu_time = sim_functions.get_time_vector(os.path.join(base_dir, "imu_data.txt"))
        attitude_time = sim_functions.get_time_vector(os.path.join(base_dir, "attitude_data.txt"))

        self.sensor_events = []
        self.sensor_events.extend([(time, 'gps') for time in gps_time])
        self.sensor_events.extend([(time, 'imu') for time in imu_time])
        self.sensor_events.extend([(time, 'attitude') for time in attitude_time])

        self.sensor_events.sort(key=lambda x: x[0])

        self.publish_data()
            

    # Asi no va a publicar el ultimo dato (como hay unos 30000 datos, tampoco pasa nada)
    def publish_data(self):

        i = 0
        while i < len(self.sensor_events) - 1:

            timestamp, sensor_id = self.sensor_events[i]
            next_timestamp = self.sensor_events[i + 1][0]

            msg = Header()
            msg.stamp.sec = int(timestamp)
            msg.frame_id = sensor_id

            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing event: {sensor_id} @ {timestamp}")

            sleep_time = next_timestamp - timestamp

            if sleep_time > 0:
                self.get_logger().debug(f"Sleeping for {sleep_time} seconds")
                time.sleep(sleep_time)
        
            i += 1



def main(args=None):
    rclpy.init(args=args)

    node = SIM_CORE()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()