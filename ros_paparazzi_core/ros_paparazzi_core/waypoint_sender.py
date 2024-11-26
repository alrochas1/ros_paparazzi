# This node send the waypoint in the txt to the raspy
# TODO: Review the service protocol (the txt logic has been moved to data_provider)
# TODO: Maybe made the number of waypoints to send dynamic
# TODO: Improve the way of managing the types (using float or int, not both)

import time
import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
from ros_paparazzi_interfaces.msg import Waypoint
from ros_paparazzi_interfaces.srv import GetWaypoint

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# Probably not needed
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)


class Waypoint_Sender(Node):

    def __init__(self):
        super().__init__('Waypoint_Sender')
        self.publisher = self.create_publisher(Waypoint, 'datalink_gps', qos_profile)

        # The node will send the waypoint just one time
        self.client = self.create_client(GetWaypoint, 'get_waypoint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for data provider service...')

        self.send_waypoint()
        


    def send_waypoint(self):
        request = GetWaypoint.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)


    # Response from the service
    def handle_response(self, future):
        try:
            response = future.result()
            msg = Waypoint()
            msg.gps.latitude = float(response.latitude)
            msg.gps.longitude = float(response.longitude)
            msg.gps.altitude = float(response.altitude)
            msg.wp_id = response.wp_id

            if self.wait_subscribers():
                self.publisher.publish(msg)
                self.get_logger().info(f'Published waypoint: lat={msg.gps.latitude}, lon={msg.gps.longitude}')
            
        except Exception as e:
            self.get_logger().error(f"Failed to get waypoint: {e}")
        finally:
            self.get_logger().info("Shutting down")
            rclpy.shutdown()


    # Function to wait for the raspy suscriber before publishing
    def wait_subscribers(self, timeout=5):
        elapsed_time = 0
        while self.publisher.get_subscription_count() == 0:
            self.get_logger().info('Waiting for subscribers...')
            time.sleep(0.5)
            elapsed_time += 0.5
            if elapsed_time >= timeout:
                self.get_logger().warning('No subscribers connected after waiting.')
                return False
        return True

    # ------------------------------------------------------------------------------------------


def main(args=None):

    rclpy.init(args=args)
    node = Waypoint_Sender()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
