from threading import Thread

from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_core.ui.ros_nodes import send_waypoint, request_home

from bokeh.plotting import figure

from pyproj import Transformer, CRS
from paramiko import SSHClient, AutoAddPolicy


P0_SANTILLANA = (40.706421, -3.87)
P0_UCM = (40.449912, -3.730109)
crs = CRS.from_epsg(3857)
transformer = Transformer.from_crs(crs.geodetic_crs, crs)

def plot_map():
    (x_min, y_min) = transformer.transform(*P0_UCM)
    (x_max, y_max) = (x_min + 500, y_min + 500)
    p = get_map(x_min, y_min, x_max, y_max)
    return p

def get_map(x_min, y_min, x_max, y_max):
    p = figure(title='Posición', x_range=(x_min, x_max), y_range=(y_min, y_max), x_axis_type="mercator", y_axis_type="mercator", width=600, height=500)
    p.add_tile("OSM")
    return p



def wpButton_Click():

    Thread(target=send_waypoint, daemon=True).start()


def home_button_Click():

    Thread(target=request_home, daemon=True).start()


# # Raspy config
# SSH_HOST = "raspy_wifi"
# SSH_USER = "ucmrospy"
# SSH_ROUTE = "/home/ucmrospy/ROS2/ros_ws/src/ros_paparazzi"
# SSH_PRIVATE_KEY_PATH = "~/.ssh/id_ed25519"

# def raspy_button_Click():
#     try: 
#         client = SSHClient()
#         client.set_missing_host_key_policy(AutoAddPolicy())
#         client.connect(SSH_HOST, username=SSH_USER, key_filename=SSH_PRIVATE_KEY_PATH)

#         command = f"""
#         cd {SSH_ROUTE}
#         docker compose up raspy
#         """
#         stdin, stdout, stderr = client.exec_command(command)

#         # print(stdout.read().decode())
#         # print(stderr.read().decode())

#         client.close()
#         gcs_data.raspy_status = True

#         print("Nodo lanzado en la Raspberry.")
#     except Exception as e:
#         gcs_data.raspy_status = False
#         print(f"Error al lanzar el nodo: {e}")



def coordinated_changed(coord):

    ValueError_message = f"Value not valid. Please use a number"
    ExceptionError_message = "Unexpected error: {e}. Value of 'new': {new}"

    def x_changed(attrname, old, new):
        try:
            x = float(new)
            gcs_data.waypoint_data.update(x, None, None)
        except ValueError:
            print(ValueError_message)
        except Exception as e:
            print(ExceptionError_message.format(e=e, new=new))
    
    def y_changed(attrname, old, new):
        try:
            y = float(new)
            gcs_data.waypoint_data.update(None, y, None) 
        except ValueError:
            print(ValueError_message)
        except Exception as e:
            print(ExceptionError_message.format(e=e, new=new))

    def wp_changed(attrname, old, new):
        try:
            wp = int(new)
            gcs_data.waypoint_data.update(None, None, wp) 
        except ValueError:
            print(ValueError_message)
        except Exception as e:
            print(ExceptionError_message.format(e=e, new=new))

    if coord == "x":
        return x_changed
    elif coord == "y":
        return y_changed
    else:
        return wp_changed
