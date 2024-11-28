# TODO: Implement HOME topic

import time
from rclpy.node import Node

from ros_paparazzi_interfaces.msg import Waypoint
from ros_paparazzi_interfaces.srv import GetWaypoint
from sensor_msgs.msg import NavSatFix

from ros_paparazzi_core.aux.geo_tools import ltp_to_wgs84
from ros_paparazzi_core.data import gcs_data


# TEMPORAL
# origin_lat = 40.4509250
# origin_lon = -3.7271889


class Waypoint_Service(Node):

    def __init__(self):
        super().__init__('Waypoint_Service')
        self.srv = self.create_service(GetWaypoint, 'get_waypoint', self.send_waypoint)
        self.publisher = self.create_publisher(Waypoint, 'waypoints/datalink', 10)
        self.suscriber = self.create_subscription(NavSatFix, 'waypoints/home', self.home_callback, 10)


    def home_callback(self, msg):
        gcs_data.origin =  [msg.latitude, msg.longitude, msg.altitude]
        

    def send_waypoint(self, request, response):

        x = request.x
        y = request.y
        wp_id = request.wp_id

        response = GetWaypoint.Response()
        response.ack = False

        try:

            msg = Waypoint()
            msg.wp_id = wp_id

            if wp_id != 0:
                [latitude, longitude] = ltp_to_wgs84(gcs_data.origin[0], gcs_data.origin[1], x, y)
                msg.gps.latitude = float(latitude)
                msg.gps.longitude = float(longitude)
                msg.gps.altitude = float(650000)
                

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

