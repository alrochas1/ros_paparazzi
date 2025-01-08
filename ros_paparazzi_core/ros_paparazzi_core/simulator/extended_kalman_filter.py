# This one is for implementing a Extended Kalman Filter using the data for the simulation

# TODO: Check if the IMU data is correct (with the orientation)
# TODO: Check the estimated position (currently is OK).

import rclpy
from rclpy.node import Node

from ros_paparazzi_interfaces.msg import Waypoint, KalmanUpdate, KalmanPredict 

from ros_paparazzi_core.aux import geo_tools
from ros_paparazzi_core.data import gcs_data


import numpy as np

or_x = gcs_data.origin[0]; or_y = gcs_data.origin[1]

# For the filter
R2_IMU = 2.5e-4 # Varianza de la IMU
RP = 0.05          # Varianza de la Posición del GPS
RV = 1          # Varianza de la Velocidad del GPS
RT = 10         # Varianza de la Actitud


class SIM_Kalman(Node):

    def __init__(self):
        super().__init__('SIM_Extended_Kalman')

        self.Kalman_publisher = self.create_publisher(Waypoint, 'waypoints/telemetry_gps', 10)
        self.PREDICT_subscription = self.create_subscription(KalmanPredict, 'kalman/predict', self.kalman_predict, 10)
        self.UPDATE_subscription = self.create_subscription(KalmanUpdate, 'kalman/update', self.kalman_update, 10)

        self.kalman_init()

    def kalman_init(self):

        dt = 0.3
        rho = R2_IMU # * pow(dt, 4)

        self.Q = np.eye(5) * rho
        self.R = np.eye(5) * 0.1
        self.P = np.eye(5) * 100
        self.H = np.eye(5)
        self.R = np.matrix([
              [RP, 0,   0,   0,  0],
              [0, RP,   0,   0,  0],
              [0,  0,  RV,   0,  0],
              [0,  0,   0,  RV,  0],
              [0,  0,   0,   0,  RT]
        ])

        self.X = np.zeros((5, 1))
        self.Y = np.zeros((5, 1))



    # Modelo dinámico no lineal
    def f(self, X, U, dt):
        ax, ay, az = U[0], U[1], U[2]
        theta = X[4]

        return np.array([
            X[0] + X[2] * dt,
            X[1] + X[3] * dt,
            X[2] + (np.cos(theta) * ax - np.sin(theta) * ay) * dt,
            X[3] + (np.sin(theta) * ax + np.cos(theta) * ay) * dt,
            X[4] + az * dt
        ]).reshape(-1, 1)


    # Modelo de observación
    def h(self, X):
        return X


    # Calcula los "jacobianos" F y H
    def jacobian_F(self, X, U, dt):

        theta = float(X[4])
        ax, ay = float(U[0]), float(U[1])

        # F (f respecto a X)
        F = np.array([
            [1, 0, dt, 0, 0],
            [0, 1, 0, dt, 0],
            [0, 0, 1, 0, -(np.sin(theta) * ax + np.cos(theta) * ay) * dt],
            [0, 0, 0, 1, +(np.cos(theta) * ax - np.sin(theta) * ay) * dt],
            [0, 0, 0, 0, 1]
        ])

        return F
    

    def kalman_predict(self, msg):
        # dt = 0.008    # Provisional

        dt = msg.dt
        ax = msg.imu.x / 1024.0
        ay = msg.imu.y / 1024.0
        az = msg.imu.y / 1024.0
        U = np.array([ax, ay, az])

        # Predicción del estado: X = f(X, U)
        self.X = self.f(self.X, U, dt)

        F = self.jacobian_F(self.X, U, dt)
        self.P = F @ self.P @ F.T + self.Q
        # self.publish_state()

    def kalman_update(self, msg):

        x, y = geo_tools.wgs84_to_ltp(or_x, or_y, msg.latitude, msg.longitude)
        # CAMBIAR TODO ESTO
        vx = msg.gps_speed.x; vy = msg.gps_speed.y
        self.Y[0] = float(x)
        self.Y[1] = float(y)
        self.Y[2] = float(vx)
        self.Y[3] = float(vy)
        self.Y[4] = msg.theta


        # K = PH^T(HPH^T + R)^-1
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # X = X + K(Y - h(X))
        Y_pred = self.h(self.X)
        self.X = self.X + K @ (self.Y - Y_pred)

        # P = (I - KH)P
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P
        self.publish_state()


    def publish_state(self):

        lat, lon = geo_tools.ltp_to_wgs84(or_x, or_y, self.X[0], self.X[1])

        state = Waypoint()
        state.gps.latitude = float(lat)
        state.gps.longitude = float(lon)
        state.gps.altitude = 650.0    # Por defecto
        state.wp_id = 0

        self.Kalman_publisher.publish(state) 
        self.get_logger().info(f'Publishing Telemetry_Data: [{state.gps.latitude:.7f}, {state.gps.longitude:.7f}]')


def main(args=None):
    rclpy.init(args=args)

    node = SIM_Kalman()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





