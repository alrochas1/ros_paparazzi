// This is the node that will run on the raspberry
// Written in C++ for testing (not working yet)

#include "rclcpp/rclcpp.hpp"

#include "ros_paparazzi_cpp/raspy_publisher_C.h"
#include "ros_paparazzi_cpp/com/paparazzi_receive.h"
// #include "ros_paparazzi_cpp/com/paparazzi_send.h"

#include <thread>

#define SR_WAYPOINT 0x57     // W
#define SR_HOME 0x48         // H

#define PORT "/dev/serial0"

std::shared_ptr<RaspyPublisher> node_ros2;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    node_ros2 = std::make_shared<RaspyPublisher>();

    std::thread serial_thread(serial_reader);

    // auto paparazzi_time = std::make_shared<TIME_THREAD>(node->get_logger());
    // std::thread time_thread(&TIME_THREAD::run, paparazzi_time, std::ref(autopilot_data::tiempo));

    // auto paparazzi_receive = std::make_shared<PPZI_TELEMETRY>(node->get_logger(), PORT);
    // std::thread receive_thread(&PPZI_TELEMETRY::run, paparazzi_receive);

    rclcpp::spin(node_ros2);
    serial_thread.join();
    rclcpp::shutdown();

    // time_thread.join();
    // receive_thread.join();

    return 0;
}