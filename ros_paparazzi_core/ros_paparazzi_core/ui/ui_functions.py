from threading import Thread

from ros_paparazzi_core.data import gcs_data
from ros_paparazzi_core.ui.ros_nodes import send_waypoint


def wpButtonClick():

    Thread(target=send_waypoint, daemon=True).start()


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
