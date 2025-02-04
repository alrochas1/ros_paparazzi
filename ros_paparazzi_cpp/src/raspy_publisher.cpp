#include <rclcpp/rclcpp.hpp>

#include <ros_paparazzi_cpp/com/paparazzi_receive.h>
#include <ros_paparazzi_interfaces/msg/waypoint.hpp>

#include <mutex>
#include <thread>
#include <queue>
#include <variant> // Para manejar múltiples tipos de datos


// Tipo genérico para manejar ambos tipos de datos
using SensorData = std::variant<TelemetryData, GpsData, ImuData>;

class RaspyPublisher : public rclcpp::Node {
public:
    RaspyPublisher() : Node("Raspy_Publisher") {
        telemetry_publisher_ = this->create_publisher<ros_paparazzi_interfaces::msg::Waypoint>("waypoints/telemetry_gps", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        telemetry_thread_ = std::thread(&RaspyPublisher::telemetry_thread, this);
    }

    ~RaspyPublisher() {
        if (telemetry_thread_.joinable()) {
            telemetry_thread_.join();
        }
    }

    // -------- GENERIC CALLBACK --------
    void sensor_callback(const SensorData& data) {
        if (std::holds_alternative<TelemetryData>(data)) {
            auto telemetry_data = std::get<TelemetryData>(data);
            auto msg = ros_paparazzi_interfaces::msg::Waypoint();
            msg.gps.longitude = telemetry_data.longitud;
            msg.gps.latitude = telemetry_data.latitud;
            msg.gps.altitude = telemetry_data.altitud;
            msg.wp_id = 0;
            telemetry_publisher_->publish(msg);
            // RCLCPP_INFO(this->get_logger(), "Publicado GPS: lon=%d, lat=%d, alt=%d", telemetry_data.longitud, telemetry_data.latitud, telemetry_data.altitud);
        }
        else if (std::holds_alternative<ImuData>(data)) {
            auto imu_data = std::get<ImuData>(data);
            auto msg = sensor_msgs::msg::Imu();
            msg.orientation.x = imu_data.orientation_x;
            msg.orientation.y = imu_data.orientation_y;
            msg.orientation.z = imu_data.orientation_z;
            msg.orientation.w = imu_data.orientation_w;
            imu_publisher_->publish(msg);
            // RCLCPP_INFO(this->get_logger(), "Publicado IMU: orientation=%f, %f, %f, %f", imu_data.orientation_x, imu_data.orientation_y, imu_data.orientation_z, imu_data.orientation_w);
        }
    }

    // -------- TELEMETRY THREAD --------
    void telemetry_thread() {
        while (rclcpp::ok()) {
            std::unique_lock<std::mutex> lock(mutex_);
            data_available_.wait(lock, [this] { return !data_queue_.empty(); });

            auto data = data_queue_.front();
            data_queue_.pop();
            lock.unlock();

            sensor_callback(data);
        }
    }

    // Función para agregar los datos a la cola
    void add_sensor_data(const SensorData& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_queue_.push(data);
        data_available_.notify_one();
    }


private:
    rclcpp::Publisher<ros_paparazzi_interfaces::msg::Waypoint>::SharedPtr telemetry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::queue<SensorData> data_queue_;
    std::mutex mutex_;
    std::condition_variable data_available_;
    std::thread telemetry_thread_;
};

std::shared_ptr<RaspyPublisher> node_ros2;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    node_ros2 = std::make_shared<RaspyPublisher>();

    start_paparazzi_thread();

    rclcpp::spin(node_ros2);
    rclcpp::shutdown();
    return 0;
}
