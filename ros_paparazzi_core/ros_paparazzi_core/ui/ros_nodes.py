from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_core.nodes.telemetry_receiver import Telemetry_Receiver
from ros_paparazzi_core.nodes.waypoint_button import Waypoint_Button
from ros_paparazzi_core.nodes.home_button import Home_Button
from ros_paparazzi_core.nodes.waypoint_service import Waypoint_Service

from ros_paparazzi_core.ui.ui_elements import get_terminal_manager

from bokeh.plotting import curdoc


def start_nodes():
    
    executor = MultiThreadedExecutor()
    
    telemetry_node = Telemetry_Receiver()
    waypoint_service = Waypoint_Service()

    executor.add_node(telemetry_node)
    executor.add_node(waypoint_service)

    Thread(target=executor.spin, daemon=True).start()

# TODO: Maybe join this two functions some way?
def send_waypoint():
    node = Waypoint_Button()
    future = node.send_waypoint()
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    print(response)
    # TODO: Do something with the response
    if response:
        gcs_data.terminal_data.update_terminal(f"Waypoint sent <br>")
    else: 
        gcs_data.terminal_data.update_terminal("Error sending waypoint <br>")
    node.destroy_node()


def request_home():
    node = Home_Button()
    future = node.request_home()
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    # TODO: Do something with the response
    if response:
        gcs_data.terminal_data.update_terminal(f"HOME request sent satisfactorily <br>")
    else: 
        gcs_data.terminal_data.update_terminal("Error sending HOME request <br>")
    
    node.destroy_node()