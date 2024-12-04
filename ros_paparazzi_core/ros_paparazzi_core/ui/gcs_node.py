# TODO: Maybe separate the waypoint_service to a diferent launch code

import rclpy

from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_core.ui import ui_functions
from ros_paparazzi_core.ui.ros_nodes import start_nodes
from ros_paparazzi_core.aux.geo_tools import wgs84_to_epsg


from bokeh.layouts import column, row, Spacer, gridplot
from bokeh.models import TextInput, Button, ColumnDataSource, Div
from bokeh.plotting import figure, curdoc

import numpy as np


# TEMPORAL: For testing de IMU
def calculate_imu():

    ux = np.mean(imu_source.data['x'])
    uy = np.mean(imu_source.data['x'])
    uz = np.mean(imu_source.data['x'])

    var_x = np.var(imu_source.data['x'])
    var_y = np.var(imu_source.data['y'])
    var_z = np.var(imu_source.data['z'])

    return [var_x, var_y, var_z]


def update_ui():
    [latitude, longitude, altitude] = gcs_data.telemetry_data.recover()
    v1x.value = f"Latitude={latitude:.4f}"
    v1y.value = f"Longitude={longitude:.4f}"

    # [x, y] = wgs84_to_epsg(gcs_data.origin[0], gcs_data.origin[1])
    # map_source.data = dict(x=[x], y=[y])

    [origin_x, origin_y] = wgs84_to_epsg(gcs_data.origin[0], gcs_data.origin[1])
    [gps_x, gps_y] = wgs84_to_epsg(gcs_data.gps_data[0], gcs_data.gps_data[1])
    [vehicle_x, vehicle_y] = wgs84_to_epsg(latitude, longitude)

    map_source.data = dict(
        origin_x=[origin_x], origin_y=[origin_y], 
        gps_x=[gps_x], gps_y=[gps_y],
        vehicle_x=[vehicle_x], vehicle_y=[vehicle_y]
    )

    if gcs_data.raspy_status:   raspy_button.button_type = "success"
    else:  raspy_button.button_type = "danger" 

    terminal_output.text = gcs_data.terminal_data.recover_message()

    # This is temporal for the IMU
    if gcs_data.time > 5:
        imu_data = gcs_data.imu_data
        imu_source.stream({
            'time': [gcs_data.time],
            'x': [imu_data[0]],
            'y': [imu_data[1]],
            'z': [imu_data[2]]
        }, rollover=1000)
        var = calculate_imu()

        variance_text.text = f"""
            <b>Varianza:</b><br>
            X: {var[0]:.7f}<br>
            Y: {var[1]:.7f}<br>
            Z: {var[2]:.7f}
            """
    gcs_data.time = gcs_data.time + 0.1


# Create UI --------------------------------------------------

# Estos tres botones son de momento genericos, por si acaso
home_button = Button(label='Request HOME', width=200, height=40)
home_button.on_click(ui_functions.home_button_Click)
raspy_button = Button(label='Connect Raspberry', width=200, height=40)
# raspy_button.on_click(raspy_button_Click)
button3 = Button(label='Button 3', width=200, height=40)

map_plot = ui_functions.plot_map()
map_source = ColumnDataSource(data=dict(origin_x=[], origin_y=[], vehicle_x=[], vehicle_y=[], gps_x=[], gps_y=[]))
map_plot.scatter(x="origin_x", y="origin_y", size=12, fill_color="red", source=map_source, legend_label="Origin")
map_plot.scatter(x="vehicle_x", y="vehicle_y", size=12, fill_color="blue", source=map_source, legend_label="Vehicle Position")
map_plot.scatter(x="gps_x", y="gps_y", size=12, fill_color="green", source=map_source, legend_label="GPS Measure")
# map_source = ColumnDataSource(data=dict(x=[gcs_data.origin[0]], y=[gcs_data.origin[1]]))
# map_plot.scatter(x="x", y="y", size=12, fill_color="red", source=map_source)

# Dos plots por si acaso tambien
imu_source = ColumnDataSource(data=dict(time=[], x=[], y=[], z=[]))
imu_plot_x = figure(title="Accel X", x_axis_label="Time (s)", y_axis_label="Acceleration (m/s^2)", width=300, height=200)
imu_plot_x.line('time', 'x', source=imu_source, line_width=2, color='red')
imu_plot_y = figure(title="Accel Y", x_axis_label="Time (s)", y_axis_label="Acceleration (m/s^2)", width=300, height=200)
imu_plot_y.line('time', 'y', source=imu_source, line_width=2, color='blue')
imu_plot_z = figure(title="Accel Z", x_axis_label="Time (s)", y_axis_label="Acceleration (m/s^2)", width=300, height=200)
imu_plot_z.line('time', 'z', source=imu_source, line_width=2, color='green')
imu_plot = gridplot([[imu_plot_x, imu_plot_y, imu_plot_z]])


v1x = TextInput(value='X=0.0m', width=150, height=15)
v1y = TextInput(value='Y=0.0m', width=150, height=15)

v2x = TextInput(value='0.0', width=150, height=15)
v2y = TextInput(value='0.0', width=150, height=15)
v2z = TextInput(value='0', width=150, height=15)
v2x.on_change('value', ui_functions.coordinated_changed("x"))
v2y.on_change('value', ui_functions.coordinated_changed("y"))
v2z.on_change('value', ui_functions.coordinated_changed("wp"))

wpButton = Button(label='Send Waypoint', width=60, height=40, button_type='success')
wpButton.on_click(ui_functions.wpButton_Click)

variance_text = Div(text="<b>Varianza:</b><br>X: Calculando...<br>Y: Calculando...<br>Z: Calculando...", 
                    stylesheets=["div { font-size: 16px; color: black; }"])

terminal_output = Div(text="<b>Terminal Output:</b><br>",
            stylesheets=["div { font-size: 16px; color: black; overflow-y: scroll; height: 150px; width: 500px; border: 1px solid black; }"])

buttons_column = column(Spacer(height=100), raspy_button, home_button, button3, Spacer(height=100))
plots = column(imu_plot, variance_text)
bottom_section = row(Spacer(width=110), column(v1x, v1y), column(v2x, v2y, v2z, Spacer(height=18), wpButton))


layout = column(row(
    Spacer(width=25),
    buttons_column, 
    Spacer(width=25),
    map_plot,
    column(plots, Spacer(height=40), bottom_section)),
    terminal_output
)


rclpy.init()
start_nodes()


curdoc().add_root(layout)
curdoc().add_periodic_callback(update_ui, 100)
