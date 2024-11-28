# TODO: Maybe separate the waypoint_service to a diferent launch code

import rclpy

from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_core.ui.ros_nodes import start_nodes
from ros_paparazzi_core.ui.ui_functions import coordinated_changed, wpButtonClick

from bokeh.layouts import column, row
from bokeh.models import TextInput, Button
from bokeh.plotting import curdoc


def update_ui():
    [latitude, longitude, altitude] = gcs_data.telemetry_data.recover()
    v1x.value = f"Latitude={latitude:.4f}"
    v1y.value = f"Longitude={longitude:.4f}"



v1x = TextInput(value='X=0.0m', width=150, height=15)
v1y = TextInput(value='Y=0.0m', width=150, height=15)

v2x = TextInput(value='0.0', width=150, height=15)
v2y = TextInput(value='0.0', width=150, height=15)
v2z = TextInput(value='2', width=150, height=15)
v2x.on_change('value', coordinated_changed("x"))
v2y.on_change('value', coordinated_changed("y"))
v2z.on_change('value', coordinated_changed("wp"))

wpButton = Button(label='Send Waypoint', width=60, height=40, button_type='success')
wpButton.on_click(wpButtonClick)

column1 = column(v1x, v1y)
column2 = column(v2x, v2y, v2z, wpButton)
layout = row(column1, column2)


rclpy.init()
start_nodes()

curdoc().add_root(layout)
curdoc().add_periodic_callback(update_ui, 100)
