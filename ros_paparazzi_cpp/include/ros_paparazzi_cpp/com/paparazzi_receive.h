#ifndef PAPARAZZI_RECEIVE_H
#define PAPARAZZI_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif

void start_paparazzi_thread();

#ifdef __cplusplus
}
#endif

#endif // PAPARAZZI_RECEIVE_H

// Definicion de tipos datos

extern struct SensorData;
extern std::shared_ptr<RaspyPublisher> node_ros2; 


struct TelemetryData {
    int wp_id;
    long longitud;
    long latitud;
    long altitud;
};


struct GpsData {
    int wp_id;
    long longitud;
    long latitud;
    long altitud;
};

struct ImuData {
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
};






