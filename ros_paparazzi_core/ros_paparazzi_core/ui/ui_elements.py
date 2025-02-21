import numpy as np

from bokeh.models import Div

# Icono de Flecha 
arrow_size = 3.5
arrow_x = [i / arrow_size for i in [0, -0.7, 1, -0.7]]
arrow_y = [i / arrow_size for i in [0, -1, 0, 1]]

def update_marker(source, pos_x, pos_y, yaw):

    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

    rotated_x = [cos_yaw * x - sin_yaw * y + pos_x for x, y in zip(arrow_x, arrow_y)]
    rotated_y = [sin_yaw * x + cos_yaw * y + pos_y for x, y in zip(arrow_x, arrow_y)]

    # Actualizar la fuente de datos en Bokeh
    source = dict(xs=[rotated_x], ys=[rotated_y])
    return source



# NOT USED
class TerminalManager:
    def __init__(self):
        self.terminal_output = Div(
            text="<b>Terminal Output:</b><br>",
            stylesheets=["div { font-size: 16px; color: black; overflow-y: scroll; height: 150px; border: 1px solid black; }"]
        )

    def log(self, message):
        self.terminal_output.text += f"{message}<br>"
    
    def get_component(self):
        return self.terminal_output
    

_terminal_manager_instance = None

def get_terminal_manager():
    global _terminal_manager_instance
    if _terminal_manager_instance is None:
        _terminal_manager_instance = TerminalManager()
    return _terminal_manager_instance
