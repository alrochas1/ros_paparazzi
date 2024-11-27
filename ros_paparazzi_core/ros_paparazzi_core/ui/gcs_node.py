# TODO: Maybe separate the waypoint_service to a diferent launch code

from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros_paparazzi_core.nodes.telemetry_receiver import Telemetry_Receiver
from ros_paparazzi_core.nodes.waypoint_button import Waypoint_Button
from ros_paparazzi_core.nodes.waypoint_sender import Waypoint_Sender
from ros_paparazzi_core.data import gcs_data

from bokeh.layouts import column, row
from bokeh.models import TextInput, Button
from bokeh.plotting import curdoc




# Function to start the nodes ----------------------------------

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
    # if response:
    #     print("OK")
    node.destroy_node()
    

# Function to interact with the UI ----------------------------------

def update_ui():
    [latitude, longitude, altitude] = gcs_data.telemetry_data.recover()
    v1x.value = f"Latitude={latitude:.4f}"
    v1y.value = f"Longitude={longitude:.4f}"


def wpButtonClick():

    gcs_data.waypoint_data.update(30.0, 30.0, 5)
    Thread(target=send_waypoint, daemon=True).start()
    

# --------------------------------------------------------------------

v1x = TextInput(value='X=0.0m', width=150, height=15)
v1y = TextInput(value='Y=0.0m', width=150, height=15)

v2x = TextInput(value='0.0', width=150, height=15)
v2y = TextInput(value='0.0', width=150, height=15)

wpButton = Button(label='Send Waypoint', width=60, height=40, button_type='success')
wpButton.on_click(wpButtonClick)

column1 = column(v1x, v1y)
column2 = column(v2x, v2y, wpButton)
layout = row(column1, column2)


rclpy.init()
start_nodes()

curdoc().add_root(layout)
curdoc().add_periodic_callback(update_ui, 100)
