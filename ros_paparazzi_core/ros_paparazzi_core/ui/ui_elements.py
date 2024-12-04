from bokeh.models import Div

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
