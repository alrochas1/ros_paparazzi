#!/bin/bash

# TEMPORAL: Connect to the Raspy and launch the node. 
# You need to have everything configured as I do


ROS_ROUTE=/home/ucmrospy/ROS2/ros_ws/src/ros_paparazzi
SSH_HOST=raspy_wifi

# Ctrl+C
cleanup() {
    echo "Stoping containers ..."
    ssh $SSH_HOST "cd $ROS_ROUTE && docker compose down"
    exit 0
}

trap cleanup SIGINT

# Launch node
ssh $SSH_HOST << EOF
cd $ROS_ROUTE
docker compose up raspy
EOF


echo "Docker up. Press Ctrl+C to stop."
while true; do sleep 1; done
