import os
import subprocess
import sys

def main():

    bokeh_app_path = os.path.join(
    os.path.dirname(__file__), 
    "../ui/gcs_node.py" 
)


    command = [
        sys.executable,
        "-m", "bokeh", "serve", bokeh_app_path,
        "--allow-websocket-origin=*"
    ]

    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to init Bokeh Serve: {e}", file=sys.stderr)

if __name__ == "__main__":
    main()
