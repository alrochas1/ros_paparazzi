#ifndef PAPARAZZI_RECEIVE_H
#define PAPARAZZI_RECEIVE_H

#include <vector>
#include <memory>

// extern std::shared_ptr<RaspyPublisher> node_ros2; // Nodo ROS2

extern void serial_reader();
extern void process_messages(const std::vector<int>& datappzz);

#endif



