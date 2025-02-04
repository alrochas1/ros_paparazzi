// Como no se mas que lo basico de C++, pongo un monton de comentarios

#include "ros_paparazzi_cpp/com/paparazzi_receive.h"
#include "ros_paparazzi_cpp/raspy_publisher_C.h"

#include <iostream>     // Para poder usar operaciones de entrada y salida (std::cout)
#include <thread>       // Para usar hilos (std::thread)
#include <mutex>        // Para usar la protección de datos compartidos mediante un mutex
#include <string>       // Para trabajar con cadenas de texto
#include <chrono>       // Para gestionar el tiempo (como usar retardos con sleep_for)
#include <vector>

// #include "serial/serial.h"
#include <libserial/SerialStream.h>


const int PPZ_START_BYTE = 0x50;
const int PPZ_TELEMETRY_BYTE = 0x01;
const int PPZ_MEASURE_BYTE = 0x02;
const int RECEIVE_INTERVAL = 1; // Intervalo de espera en segundos


std::mutex data_mutex;
std::vector<int> global_data;   // Para la lectura del puerto serie

// VARIABLES GLOBALES
std::string serial_data = "MSG_TYPE_1:123";    // Esto hay que cambiarlo
// Aqui metere mis variables de mensajes; GPS, lidar, imu, ...
std::string last_message_type1;
std::string last_message_type2;
// std::mutex data_mutex;  // Es el mutex (evidentemente)


// Funciones auxiliares (REVISAR)
int serial_byteToint(const std::vector<int>& byte_array, int length) {
    int result = 0;
    for (int i = 0; i < length; i++) {
        result |= (byte_array[i] << (8 * (length - i - 1)));
    }
    return result;
}

int calculate_signo(int sign) {
    return (sign == 1) ? -1 : 1;
}



// Funcion principal del hilo, lee el puerto serie
void serial_reader() {
    // serial::Serial my_serial("/dev/serial0", 115200, serial::Timeout::simpleTimeout(1000));

    // while (true) {
    //     if (my_serial.available()) {
    //         std::vector<uint8_t> buffer(my_serial.available());
    //         my_serial.read(buffer, buffer.size());

    //         std::lock_guard<std::mutex> lock(data_mutex);
    //         for (auto byte : buffer) {
    //             global_data.push_back(byte);
    //         }

    //         process_messages(global_data);
    //         global_data.clear();
    //     }
    // }
}



// // Función para simular la lectura del puerto serie (tambien habra que cambiarla)
// std::string read_serial() {
//     std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simula retardo de lectura
//     return serial_data; // En un caso real, aquí leeríamos del puerto serie
// }


// Función para dividir los mensajes basados en el byte de salto de línea (10 en decimal)
void process_messages(const std::vector<int>& datappzz) {
    std::vector<int> message;
    std::vector<std::vector<int>> messages;

    for (std::vector<int>::size_type i = 0; i < datappzz.size(); i++) {
        if (datappzz[i] == 10) { // Salto de línea
            if (!message.empty()) {
                messages.push_back(message);
            }
            message.clear();
        } else {
            message.push_back(datappzz[i]);
            if (i == (datappzz.size() - 1)) {
                messages.push_back(message);
            }
        }
    }

    // Procesar cada mensaje
    for (auto& msg : messages) {
        if (msg.empty()) {
            continue; // Ignorar mensajes vacíos
        }

        if (msg.size() < 2 || msg[0] != PPZ_START_BYTE) {
            std::cout << "[PPZI_RECEIVE] - NO EMPIEZA POR P" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(RECEIVE_INTERVAL));
            continue;
        }

        if (msg[1] == PPZ_TELEMETRY_BYTE) {
            if (msg.size() != 26) {
                std::cout << "[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 26, Received " << msg.size() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(RECEIVE_INTERVAL));
                continue;
            }

            // Aquí puedes procesar los datos según sea necesario, usando las funciones auxiliares
            int sign_long = msg[4];
            std::vector<int> hex_long = {msg[8], msg[7], msg[6], msg[5]};
            int longitud = calculate_signo(sign_long) * serial_byteToint(hex_long, 4);

            int sign_lat = msg[9];
            std::vector<int> hex_lat = {msg[13], msg[12], msg[11], msg[10]};
            int latitud = calculate_signo(sign_lat) * serial_byteToint(hex_lat, 4);

            int sign_alt = datappzz[14];
            if (sign_alt != 0 && sign_alt != 1) {
                sign_alt = 0;
            }
            std::vector<int> hex_alt = {datappzz[18], datappzz[17], datappzz[16], datappzz[15]};
            int altitud = calculate_signo(sign_alt) * serial_byteToint(hex_alt, 4);

            // std::vector<int> hex_d_sonar = {msg[22], msg[21], msg[20], msg[19]};
            // int d_sonar = serial_byteToint(hex_d_sonar, 4);

            // int c_sonar = msg[23];

            std::cout << "[PPZI_RECEIVE] - Dato de telemetría: Lon: " << longitud << ", Lat: " << latitud << ", Alt: " << altitud << std::endl;

            // Llamar al callback de ROS2 para publicar los datos
            if (node_ros2) {
                node_ros2->telemetry_callback(0, longitud, latitud, altitud);
            }
        }

        // Agregar aquí más condiciones para otros tipos de mensaje si es necesario
    }
}



// TODO ESTO HAY QUE COMPLETARLO
// Hilo que lee del puerto serie y actualiza las variables globales
// void serial_reader() {
//     while (true) {
//         std::string msg = read_serial();

//         std::lock_guard<std::mutex> lock(data_mutex);
//         if (msg.find("MSG_TYPE_1") != std::string::npos) {
//             last_message_type1 = msg;
//         } else if (msg.find("MSG_TYPE_2") != std::string::npos) {
//             last_message_type2 = msg;
//         }

//         std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Simula frecuencia de lectura
//     }
// }

// AQUI HABRA QUE QUITAR ESTO PARA CAMBIARLO POR EL NODO ROS2
// int main() {
//     std::thread serial_thread(serial_reader);

//     while (true) {
//         {
//             std::lock_guard<std::mutex> lock(data_mutex);
//             std::cout << "Último MSG_TYPE_1: " << last_message_type1 << std::endl;
//             std::cout << "Último MSG_TYPE_2: " << last_message_type2 << std::endl;
//         }
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }

//     serial_thread.join();
//     return 0;
// }
