#include <rclcpp/rclcpp.hpp>

#include <ros_paparazzi_interfaces/msg/waypoint.hpp>

#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>


struct TelemetryData {
    int wp_id;
    long longitud;
    long latitud;
    long altitud;
};

class RaspyPublisher : public rclcpp::Node {
public:
    RaspyPublisher() : Node("Raspy_Publisher") {
        telemetry_publisher_ = this->create_publisher<ros_paparazzi_interfaces::msg::Waypoint>("waypoints/telemetry_gps", 10);
        // Añadir el resto de publishers

        telemetry_thread_ = std::thread(&RaspyPublisher::telemetry_thread, this);
    }

    ~RaspyPublisher() {
        if (telemetry_thread_.joinable()) {
            telemetry_thread_.join();
        }
    }


    // -------------------- CALLBACKS --------------------
    void telemetry_callback(int wp_id, int longitud, int latitud, int altitud) {
        auto msg = ros_paparazzi_interfaces::msg::Waypoint();
        msg.gps.longitude = longitud;
        msg.gps.latitude = latitud;
        msg.gps.altitude = altitud;
        msg.wp_id = wp_id;

        telemetry_publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Publicado: lon=%d, lat=%d, alt=%d", longitud, latitud, altitud);
    }



    // -------------------- HILOS --------------------
    void telemetry_thread() {
        while (rclcpp::ok()) {
            std::unique_lock<std::mutex> lock(mutex_);
            data_available_.wait(lock, [this] { return !data_queue_.empty(); });

            // Tomar los datos de la cola
            auto data = data_queue_.front();
            data_queue_.pop();
            lock.unlock();

            // Publicar los datos
            telemetry_callback(data.wp_id, data.longitud, data.latitud, data.altitud);
        }
    }

    // Función para agregar los datos a la cola
    void add_telemetry_data(const TelemetryData& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        data_queue_.push(data);
        data_available_.notify_one();
    }

private:
    rclcpp::Publisher<ros_paparazzi_interfaces::msg::Waypoint>::SharedPtr telemetry_publisher_;
    std::queue<TelemetryData> data_queue_;
    std::mutex mutex_;
    std::condition_variable data_available_;
    std::thread telemetry_thread_;  // Hilo que gestiona la publicación
};

std::shared_ptr<RaspyPublisher> node_ros2;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    node_ros2 = std::make_shared<RaspyPublisher>();
    
    // Aquí en algún lugar lanzarías tu hilo en C para leer los datos del puerto serie
    // Ejemplo de cómo se lanzaría el hilo en C (no se muestra el código del hilo C aquí)
    PI_THREAD(comunicacion_ppzz);

    rclcpp::spin(node_ros2);
    rclcpp::shutdown();
    return 0;
}
