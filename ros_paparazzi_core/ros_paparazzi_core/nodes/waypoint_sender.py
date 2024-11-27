import time
from rclpy.node import Node

# from sensor_msgs.msg import NavSatFix
from ros_paparazzi_interfaces.msg import Waypoint
from ros_paparazzi_interfaces.srv import GetWaypoint

from ros_paparazzi_core.aux.geo_tools import ltp_to_wgs84

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# TEMPORAL
origin_lat = 40.4509250
origin_lon = -3.7271889


# Probably not needed
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)


class Waypoint_Sender(Node):

    def __init__(self):
        super().__init__('Waypoint_Sender')
        self.srv = self.create_service(GetWaypoint, 'get_waypoint', self.send_waypoint)
        self.publisher = self.create_publisher(Waypoint, 'datalink_gps', qos_profile)
        

    def send_waypoint(self, request, response):

        x = request.x
        y = request.y
        wp_id = request.wp_id

        response = GetWaypoint.Response()
        response.ack = False

        try:
            [latitude, longitude] = ltp_to_wgs84(origin_lat, origin_lon, x, y)

            msg = Waypoint()
            msg.gps.latitude = float(latitude)
            msg.gps.longitude = float(longitude)
            msg.gps.altitude = float(650000)
            msg.wp_id = wp_id

            if self.wait_subscribers():
                self.publisher.publish(msg)
                self.get_logger().info(f'Published waypoint: lat={msg.gps.latitude}, lon={msg.gps.longitude}')
                response.ack = True
            
        except Exception as e:
            self.get_logger().error(f"Failed to get waypoint: {e}")
        finally:
            return response


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


# def main(args=None):

#     rclpy.init(args=args)
#     node = Waypoint_Sender()
#     rclpy.spin(node)


# if __name__ == '__main__':
#     main()