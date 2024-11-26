# This node is used for receiving the GPS telemetry data 
# TODO: Store the data somewhere

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix



class Telemetry_Subscriber(Node):

    def __init__(self):
        super().__init__('Telemetry_Receiver')
        self.subscription = self.create_subscription(NavSatFix, 'telemetry_gps', self.telemetry_callback, 10)
        
    def telemetry_callback(self, msg):
        self.get_logger().info(f'Receiving data: [{msg.latitude:.7f}, {msg.longitude:.7f}, {msg.altitude:.2f}]')

    # ------------------------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    telemetry_subscriber = Telemetry_Subscriber()

    rclpy.spin(telemetry_subscriber)

    telemetry_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()