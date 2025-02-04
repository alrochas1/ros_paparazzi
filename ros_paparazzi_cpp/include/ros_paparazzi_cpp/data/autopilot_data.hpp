#ifndef AUTOPILOT_DATA_HPP
#define AUTOPILOT_DATA_HPP

#include <vector>
#include <functional>
#include <iostream>

class TelemetryData {
public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    int wp_id = 0;
    std::vector<std::function<void(const TelemetryData&)>> callbacks;

    void update(double t, double lon, double lat, double alt, int id) {
        time = t;
        longitude = lon;
        latitude = lat;
        altitude = alt;
        wp_id = id;
        send_callback();
    }

    void register_callback(std::function<void(const TelemetryData&)> callback) {
        callbacks.push_back(callback);
    }

    void send_callback() {
        for (auto& callback : callbacks) {
            callback(*this);
        }
    }

    std::vector<double> recover() const {
        return {time, longitude, latitude, altitude, static_cast<double>(wp_id)};
    }
};

class WaypointData {
public:
    int lat = 0;
    int lon = 0;
    int alt = 0;
    int wp_id = 0;

    void update(int latitude, int longitude, int altitude, int id) {
        lat = latitude;
        lon = longitude;
        alt = altitude;
        wp_id = id;
    }

    std::vector<int> recover() const {
        return {lat, lon, alt, wp_id};
    }
};

class IMUData {
public:
    int x = 0;
    int y = 0;
    int z = 0;
    double time = 0.0;
    std::vector<std::function<void(const IMUData&)>> callbacks;

    void update(double t, int x_val, int y_val, int z_val) {
        time = t;
        x = x_val;
        y = y_val;
        z = z_val;
        send_callback();
    }

    void register_callback(std::function<void(const IMUData&)> callback) {
        callbacks.push_back(callback);
    }

    void send_callback() {
        for (auto& callback : callbacks) {
            callback(*this);
        }
    }
};

class GPSData {
public:
    int lat = 0;
    int lon = 0;
    int alt = 0;
    std::vector<std::function<void(const GPSData&)>> callbacks;

    void update(int latitude, int longitude, int altitude) {
        lat = latitude;
        lon = longitude;
        alt = altitude;
        send_callback();
    }

    void register_callback(std::function<void(const GPSData&)> callback) {
        callbacks.push_back(callback);
    }

    void send_callback() {
        for (auto& callback : callbacks) {
            callback(*this);
        }
    }
};

class LidarData {
public:
    float distance = 0.0;
    float angle = 0.0;
    std::vector<std::function<void(const LidarData&)>> callbacks;

    void update(float dist, float ang) {
        distance = dist;
        angle = ang;
        send_callback();
    }

    void register_callback(std::function<void(const LidarData&)> callback) {
        callbacks.push_back(callback);
    }

    void send_callback() {
        for (auto& callback : callbacks) {
            callback(*this);
        }
    }
};


// Instancias globales
inline TelemetryData telemetry_data;
inline WaypointData waypoint_data;
inline TelemetryData home_data;
inline IMUData imu_data;
inline GPSData gps_data;
inline LidarData lidar_data;

#endif // AUTOPILOT_DATA_HPP