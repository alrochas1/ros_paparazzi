# TODO: Maybe separate the waypoint_service to a diferent launch code

import rclpy

from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_core.ui.ros_nodes import start_nodes
from ros_paparazzi_core.ui.ui_functions import coordinated_changed, wpButtonClick, plot_map

from bokeh.layouts import column, row, Spacer
from bokeh.models import TextInput, Button
from bokeh.plotting import figure
from bokeh.plotting import curdoc


def update_ui():
    [latitude, longitude, altitude] = gcs_data.telemetry_data.recover()
    v1x.value = f"Latitude={latitude:.4f}"
    v1y.value = f"Longitude={longitude:.4f}"



# Create UI

# Estos tres botones son de momento genericos, por si acaso
home_button = Button(label='Request HOME', width=100, height=40)
button2 = Button(label='Button 2', width=100, height=40)
button3 = Button(label='Button 3', width=100, height=40)

map_plot = plot_map()

# Dos plots por si acaso tambien
plot1 = figure(width=270, height=300, title="Plot 1")
plot1.line([0, 1, 2, 3], [4, 5, 6, 7], line_width=2)
plot2 = figure(width=270, height=300, title="Plot 2")
plot2.line([0, 1, 2, 3], [7, 6, 5, 4], line_width=2)


v1x = TextInput(value='X=0.0m', width=150, height=15)
v1y = TextInput(value='Y=0.0m', width=150, height=15)

v2x = TextInput(value='0.0', width=150, height=15)
v2y = TextInput(value='0.0', width=150, height=15)
v2z = TextInput(value='0', width=150, height=15)
v2x.on_change('value', coordinated_changed("x"))
v2y.on_change('value', coordinated_changed("y"))
v2z.on_change('value', coordinated_changed("wp"))

wpButton = Button(label='Send Waypoint', width=60, height=40, button_type='success')
wpButton.on_click(wpButtonClick)


buttons_column = column(Spacer(height=100), home_button, button2, button3, Spacer(height=100))
plots = row(plot1, plot2)
bottom_section = row(Spacer(width=110), column(v1x, v1y), column(v2x, v2y, v2z, Spacer(height=18), wpButton))


layout = row(
    Spacer(width=25),
    buttons_column, 
    Spacer(width=25),
    map_plot,
    column(plots, Spacer(height=40), bottom_section)
)




rclpy.init()
start_nodes()

curdoc().add_root(layout)
curdoc().add_periodic_callback(update_ui, 100)
