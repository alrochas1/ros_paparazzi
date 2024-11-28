from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros_paparazzi_core.nodes.telemetry_receiver import Telemetry_Receiver
from ros_paparazzi_core.nodes.waypoint_button import Waypoint_Button
from ros_paparazzi_core.nodes.waypoint_sender import Waypoint_Sender



def start_nodes():
    
    executor = MultiThreadedExecutor()
    
    telemetry_node = Telemetry_Receiver()
    waypoint_service = Waypoint_Sender()

    executor.add_node(telemetry_node)
    executor.add_node(waypoint_service)

    Thread(target=executor.spin, daemon=True).start()


def send_waypoint():
    node = Waypoint_Button()
    future = node.send_waypoint()
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    print(response)
    # TODO: Do something with the response
    node.destroy_node()