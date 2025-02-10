# Ros_Paparazzi

`ros_paparazzi` is a ROS 2 package that includes the functionalities to communicate with the Paparazzi autopilot system.

## Repository Structure

This repository contains two packages:

- **ros_paparazzi_core**: Contains the main Python nodes responsible for subscribing and publishing data to/from the Paparazzi autopilot system.
- **ros_paparazzi_interfaces**: Defines the custom ROS 2 message and service types used by `ros_paparazzi_core`.

## Requirements

- ROS 2 Humble (a Docker image is also available).
- A Raspberry with a SSH configured (SSH is no longer required after installation)


## Setup Instructions

Clone the repository into your ROS 2 workspace:

```bash
cd {ROS_WORKSPACE}/src
git clone {REPOSITORY}
```

For an easy installation, a `./install.sh` is included (you can also use colcon to build the package).
If you dont have ROS2 in your computer, check the docker section.


## Running the Nodes

At the moment there are four nodes:
- **Raspy_Publisher**: Manage the serial communication with the AP
- **Waypoint_Service**: Send the waypoint to the raspberry
- **Telemetry_Receiver**: Receive the telemetry info (currently position and IMU)
- **X_Button**: Multiple Nodes to respond to the buttons of the UI

![Nodes Diagram](./graphs/rosgraph3.png)

In your computer, launch the UI with
```
ros2 run ros_paparazzi_core bokeh_serve
```

For launching the node in the raspberry, a script `launch_ros_node.sh` is provided.


<!-- TODO: Add the ROS argument instructions -->


## The UI

Currently, there are two working buttons
- The Send Waypoint button will send the waypoint coordinates indicated above itself.
- The Request HOME will reset the coordinates origin
- The Connect Raspberry will launch the node in the Raspberry (not working)


## The Simulator

To replace the data sent by telemetry with the data obtained during testing, launch the simulation with: 
```
ros2 launch ros_paparazzi_core sim_launch.py
```

You will probably need to update the path from which the data is read in `sim_config.yaml`

## Docker

If you dont have ROS2 Humble installed on your computer, you can use the provided Docker image:

```bash
docker pull alrochas01/ucm-humble:latest
docker tag alrochas01/ucm-humble:latest ucm:humble-desktop
docker rmi alrochas01/ucm-humble:latest
```

Install the repository with:
```bash
docker compose up installer
```

Then for lauching the UI you can use:
```bash
docker compose up gcs
```

and for lauching the simulation (not working yet)
```bash
docker compose up simulator
```


