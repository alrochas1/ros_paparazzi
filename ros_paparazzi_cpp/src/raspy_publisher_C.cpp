// This is the node that will run on the raspberry
// Written in C++ for testing (not working yet)

#include "rclcpp/rclcpp.hpp"

#include "ros_paparazzi_interfaces/msg/waypoint.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "ros_paparazzi_cpp/data/autopilot_data.hpp"
#include "ros_paparazzi_cpp/com/paparazzi_receive.h"
#include "ros_paparazzi_cpp/com/paparazzi_send.h"

#include <thread>
#include <memory>

#define SR_WAYPOINT 0x57     // W
#define SR_HOME 0x48         // H

#define PORT "/dev/serial0"

class RaspyPublisher : public rclcpp::Node {
public:
    RaspyPublisher() : Node("Raspy_Publisher") {
        telemetry_publisher_ = this->create_publisher<ros_paparazzi_interfaces::msg::Waypoint>("waypoints/telemetry_gps", 10);
        subscriber_ = this->create_subscription<ros_paparazzi_interfaces::msg::Waypoint>(
            "waypoints/datalink", 10, std::bind(&RaspyPublisher::waypoint_callback, this, std::placeholders::_1));
        imu_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("sensors/imu", 10);
        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("sensors/gps", 10);
        lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("sensors/lidar", 10);

        autopilot_data::telemetry_data.register_callback(std::bind(&RaspyPublisher::telemetry_callback, this, std::placeholders::_1));
        autopilot_data::imu_data.register_callback(std::bind(&RaspyPublisher::imu_callback, this, std::placeholders::_1));
        autopilot_data::gps_data.register_callback(std::bind(&RaspyPublisher::gps_callback, this, std::placeholders::_1));
        autopilot_data::lidar_data.register_callback(std::bind(&RaspyPublisher::lidar_callback, this, std::placeholders::_1));

        paparazzi_send_ = std::make_shared<PPZI_DATALINK>(PORT);
        paparazzi_send_->run();

        ppzi_telemetry_ = std::make_shared<PPZI_TELEMETRY>(PORT, this->get_logger());
        ppzi_telemetry_->start();

    }

    ~RaspyPublisher() {
        // Detener el hilo de telemetría
        ppzi_telemetry_->stop();
    }

private:
    void waypoint_callback(const ros_paparazzi_interfaces::msg::Waypoint::SharedPtr msg) {
        int lat = static_cast<int>(msg->gps.latitude * 1e7);
        int lon = static_cast<int>(msg->gps.longitude * 1e7);
        int alt = static_cast<int>(msg->gps.altitude);
        int wp_id = msg->wp_id;

        RCLCPP_INFO(this->get_logger(), "Receiving data [%d]: [%.7f, %.7f]", wp_id, lat * 1e-7, lon * 1e-7);
        autopilot_data::waypoint_data.update(lat, lon, alt, wp_id);

        uint8_t msg_type = (wp_id == 0) ? SR_HOME : SR_WAYPOINT;
        paparazzi_send_->send(msg_type);
    }

    void telemetry_callback(const autopilot_data::TelemetryData &data) {
        auto msg = ros_paparazzi_interfaces::msg::Waypoint();
        msg.gps.header.stamp.sec = static_cast<int32_t>(autopilot_data::tiempo);
        msg.gps.header.stamp.nanosec = static_cast<int32_t>(1e9 * (autopilot_data::tiempo - static_cast<int32_t>(autopilot_data::tiempo)));
        msg.gps.longitude = static_cast<double>(data.longitude * 1e-7);
        msg.gps.latitude = static_cast<double>(data.latitude * 1e-7);
        msg.gps.altitude = static_cast<double>(data.altitude * 1e-7);
        msg.wp_id = static_cast<int32_t>(data.wp_id);
        telemetry_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing Telemetry_Data[%d]: [%.7f, %.7f, %.2f]", msg.wp_id, msg.gps.latitude, msg.gps.longitude, msg.gps.altitude);
    }

    void imu_callback(const autopilot_data::ImuData &data) {
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = static_cast<double>(data.x);
        msg.y = static_cast<double>(data.y);
        msg.z = static_cast<double>(data.z);
        imu_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing IMU_Data: [%.2f, %.2f, %.2f]", msg.x, msg.y, msg.z);
    }

    void gps_callback(const autopilot_data::GpsData &data) {
        auto msg = sensor_msgs::msg::NavSatFix();
        msg.longitude = static_cast<double>(data.lon * 1e-7);
        msg.latitude = static_cast<double>(data.lat * 1e-7);
        msg.altitude = static_cast<double>(data.alt * 1e-7);
        gps_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing GPS_Data: [%.7f, %.7f]", msg.latitude, msg.longitude);
    }

    void lidar_callback(const autopilot_data::LidarData &data) {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.ranges = {static_cast<float>(data.distance)};
        msg.angle_increment = static_cast<float>(data.angle);
        msg.range_min = 0.1f;
        msg.range_max = 12.0f;
        lidar_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing LIDAR_Data: %.2f m, Angle: %.2fº", data.distance, data.angle);
    }

    rclcpp::Publisher<ros_paparazzi_interfaces::msg::Waypoint>::SharedPtr telemetry_publisher_;
    rclcpp::Subscription<ros_paparazzi_interfaces::msg::Waypoint>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;

    std::shared_ptr<PPZI_DATALINK> paparazzi_send_;
    std::shared_ptr<PPZI_TELEMETRY> ppzi_telemetry_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RaspyPublisher>();

    auto paparazzi_time = std::make_shared<TIME_THREAD>(node->get_logger());
    std::thread time_thread(&TIME_THREAD::run, paparazzi_time, std::ref(autopilot_data::tiempo));

    auto paparazzi_receive = std::make_shared<PPZI_TELEMETRY>(node->get_logger(), PORT);
    std::thread receive_thread(&PPZI_TELEMETRY::run, paparazzi_receive);

    rclcpp::spin(node);
    rclcpp::shutdown();

    time_thread.join();
    receive_thread.join();

    return 0;
}