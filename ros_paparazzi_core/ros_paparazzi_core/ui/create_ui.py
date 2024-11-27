from threading import Thread
import rclpy
from ros_paparazzi_core.nodes.gcs_node import Telemetry_Subscriber
from bokeh.layouts import column
from bokeh.models import TextInput
from bokeh.plotting import curdoc

from ros_paparazzi_core.data import gcs_data

v1x = TextInput(value='X=0.0m', width=150, height=15)
v1y = TextInput(value='Y=0.0m', width=150, height=15)

layout = column(v1x, v1y)

def start_ros_node():
    rclpy.init()
    node = Telemetry_Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

def update_ui():
    [latitude, longitude, altitude] = gcs_data.telemetry_data.recover()
    v1x.value = f"Latitude={latitude:.4f}"
    v1y.value = f"Longitude={longitude:.4f}"

Thread(target=start_ros_node, daemon=True).start()

curdoc().add_root(layout)
curdoc().add_periodic_callback(update_ui, 100)
