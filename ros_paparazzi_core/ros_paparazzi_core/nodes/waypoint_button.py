from rclpy.node import Node
from ros_paparazzi_interfaces.srv import GetWaypoint

from ros_paparazzi_core.data import gcs_data

class Waypoint_Button(Node):

    def __init__(self):
        super().__init__('Waypoint_Button')

        # The node will send the waypoint just one time
        self.client = self.create_client(GetWaypoint, 'get_waypoint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for data provider service...')


    def send_waypoint(self):

        request = GetWaypoint.Request()
        [request.x, request.y, request.wp_id] = gcs_data.waypoint_data.recover()
        return self.client.call_async(request)
