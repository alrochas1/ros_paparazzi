from rclpy.node import Node
from ros_paparazzi_interfaces.srv import GetWaypoint

# from ros_paparazzi_core.data import gcs_data

class Home_Button(Node):

    def __init__(self):
        super().__init__('Home_Button')

        # The node will send the waypoint just one time
        self.client = self.create_client(GetWaypoint, 'get_waypoint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for data provider service...')


    def request_home(self):

        request = GetWaypoint.Request()
        request.wp_id = 0
        return self.client.call_async(request)


    # Response from the service
    def handle_response(self, future):
        # response = future.result()
        # if response.ack:
        #     print("OK")
        # else:
        #     print("Error")
        print("Test")