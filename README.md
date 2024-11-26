# Ros_Paparazzi

`ros_paparazzi` is a ROS 2 package that includes the functionalities to communicate with the Paparazzi autopilot system.

## Repository Structure

This repository contains two packages:

- **ros_paparazzi_core**: Contains the main Python nodes responsible for subscribing and publishing data to/from the Paparazzi autopilot system.
- **ros_paparazzi_interfaces**: Defines the custom ROS 2 message and service types used by `ros_paparazzi_core`.

## Requirements

- ROS 2 Humble
- A Raspberry with a SSH configured


## Setup Instructions

Clone the repository into your ROS 2 workspace:

```bash
cd {ROS_WORKSPACE}/src
git clone {REPOSITORY}
```

For an easy install, a `./install.sh` is included (you can also use colcon to build the package).


## Running the Nodes

At the moment there are four nodes:
- **raspy_publisher**: Manage the serial communication with the AP
- **waypoint_sender**: Send the waypoint (currently from the txt) to the raspberry
- **telemetry**: Receive the telemetry GPS info
- **data_provider**: Provide the data from the txt (temporal)

![Nodes Diagram](rosgraph2.png)

<!-- At the moment there are two nodes that can be runned. -->

<!-- ![Nodes Diagram](rosgraph.png)


For lauching the node in the Raspberry, run (inside a docker):

```
ros2 run ros_paparazzi_core raspy
```

And in your computer, run:

```
ros2 run ros_paparazzi_core computer -->

Launch files are provided to make running the nodes easier.
Use `main_launch.py` for launching the nodes that will run indefinitely

```
ros2 launch ros_paparazzi_core main_launch.py
```

Use `send_launch.py` to send a waypoint to the autopilot

```
ros2 launch ros_paparazzi_core send_launch.py
```

For launching the node in the raspberry, a script `launch_ros_node.sh` is provided.

If you have everything connected, you should see the waypoint defined in the `data_LTP.txt` move


TODO: Add the ROS argument instructions



