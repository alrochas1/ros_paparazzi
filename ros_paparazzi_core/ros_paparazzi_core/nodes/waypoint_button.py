from rclpy.node import Node
# from ros_paparazzi_interfaces.msg import Waypoint
from ros_paparazzi_interfaces.srv import GetWaypoint

from ros_paparazzi_core.data import gcs_data

class Waypoint_Button(Node):

    def __init__(self):
        super().__init__('Waypoint_Button')

        # The node will send the waypoint just one time
        self.client = self.create_client(GetWaypoint, 'get_waypoint')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for data provider service...')

        # self.send_waypoint()
        # self.destroy_node()


    def send_waypoint(self):

        request = GetWaypoint.Request()
        [request.x, request.y, request.wp_id] = gcs_data.waypoint_data.recover()
        return self.client.call_async(request)
        # future.add_done_callback(self.handle_response)


    # Response from the service
    def handle_response(self, future):
        # response = future.result()
        # if response.ack:
        #     print("OK")
        # else:
        #     print("Error")
        print("Test")
    

   
# def main(args=None):
#     rclpy.init(args=args)
#     node = Waypoint_Button()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

