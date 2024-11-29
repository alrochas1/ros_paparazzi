# TODO: Maybe separate the waypoint_service to a diferent launch code

import rclpy

from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_core.ui.ros_nodes import start_nodes
from ros_paparazzi_core.ui.ui_functions import coordinated_changed, wpButton_Click, home_button_Click, plot_map
from ros_paparazzi_core.aux.geo_tools import wgs84_to_epsg

from bokeh.layouts import column, row, Spacer
from bokeh.models import TextInput, Button, ColumnDataSource
from bokeh.plotting import figure, curdoc


def update_ui():
    [latitude, longitude, altitude] = gcs_data.telemetry_data.recover()
    v1x.value = f"Latitude={latitude:.4f}"
    v1y.value = f"Longitude={longitude:.4f}"

    [x, y] = wgs84_to_epsg(gcs_data.origin[0], gcs_data.origin[1])
    source.data = dict(x=[x], y=[y])

    if gcs_data.raspy_status:   raspy_button.button_type = "success"
    else:  raspy_button.button_type = "danger" 

    imu_data = gcs_data.imu_data
    imu_source.data['time'].append(gcs_data.time)
    imu_source.data['x'].append(imu_data[0])
    imu_source.data['y'].append(imu_data[1])
    imu_source.data['z'].append(imu_data[2])
    gcs_data.time = gcs_data.time + 0.1


# Create UI

# Estos tres botones son de momento genericos, por si acaso
home_button = Button(label='Request HOME', width=200, height=40)
home_button.on_click(home_button_Click)
raspy_button = Button(label='Connect Raspberry', width=200, height=40)
# raspy_button.on_click(raspy_button_Click)
button3 = Button(label='Button 3', width=200, height=40)

map_plot = plot_map()
source = ColumnDataSource(data=dict(x=[gcs_data.origin[0]], y=[gcs_data.origin[1]]))
map_plot.scatter(x="x", y="y", size=12, fill_color="red", source=source)

# Dos plots por si acaso tambien
imu_source = ColumnDataSource(data=dict(time=[], x=[], y=[], z=[]))
imu_plot = figure(title="IMU Data", x_axis_label="Time", y_axis_label="Acceleration (m/s^2)")
imu_plot.line('time', 'x', source=imu_source, line_width=2, color='red', legend_label='Accel X')
imu_plot.line('time', 'y', source=imu_source, line_width=2, color='blue', legend_label='Accel Y')
imu_plot.line('time', 'z', source=imu_source, line_width=2, color='green', legend_label='Accel Z')
imu_plot.legend.location = "top_left"


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
wpButton.on_click(wpButton_Click)


buttons_column = column(Spacer(height=100), raspy_button, home_button, button3, Spacer(height=100))
plots = row(imu_plot, plot2)
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
