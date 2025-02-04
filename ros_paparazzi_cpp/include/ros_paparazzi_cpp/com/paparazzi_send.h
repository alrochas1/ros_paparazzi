#include <rclcpp/rclcpp.hpp>
// #include <serial/serial.h>
#include <vector>
#include <chrono>
#include <thread>

#include "ros_paparazzi_cpp/data/autopilot_data.hpp"

#define COM_START_BYTE 0x52 // 'R'
#define SR_WAYPOINT 0x57    // 'W'
#define SR_HOME 0x48        // 'H'

class PPZI_DATALINK {
public:
    PPZI_DATALINK(const std::string& port = "/dev/ttyUSB0") : port_(port), baud_rate_(115200) {
        try {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_rate_);
            serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
            serial_.open();
            RCLCPP_INFO(rclcpp::get_logger("PPZI_DATALINK"), "Conexión establecida con el autopiloto.");
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(rclcpp::get_logger("PPZI_DATALINK"), "Error al abrir el puerto serie.");
        }
    }

    void send(uint8_t msg_id) {
        if (!serial_.isOpen()) {
            RCLCPP_WARN(rclcpp::get_logger("PPZI_DATALINK"), "Puerto serie no disponible.");
            return;
        }

        auto [lat, lon, alt, wp_id] = autopilot_data::waypoint_data::recover();
        RCLCPP_INFO(rclcpp::get_logger("PPZI_DATALINK"), "[PPZI_SEND] - Coordenadas enviadas: [%.7f, %.7f]", lat * 1e-7, lon * 1e-7);
        
        std::vector<uint8_t> message = {COM_START_BYTE, msg_id, static_cast<uint8_t>(wp_id)};
        appendData(message, lat, lon, alt);
        appendChecksum(message);

        serial_.write(message);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    void close() {
        if (serial_.isOpen()) {
            serial_.close();
            RCLCPP_INFO(rclcpp::get_logger("PPZI_DATALINK"), "Conexión cerrada.");
        }
    }

private:
    std::string port_;
    int baud_rate_;
    serial::Serial serial_;

    void appendData(std::vector<uint8_t>& msg, int lat, int lon, int alt) {
        for (int val : {lat, lon, alt}) {
            for (size_t i = 0; i < sizeof(int); i++) {
                msg.push_back(static_cast<uint8_t>(val >> (i * 8)));
            }
        }
    }

    void appendChecksum(std::vector<uint8_t>& msg) {
        uint16_t checksum = 0;
        for (auto byte : msg) checksum += byte;
        msg.push_back(checksum & 0xFF);
        msg.push_back((checksum >> 8) & 0xFF);
    }
};