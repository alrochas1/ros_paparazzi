# This one is for implementing a Kalman Filter using the data for the simulation

#TODO: Check if the IMU data is correct (with the orientation)
#TODO: Check the estimated position (that currently is pretty bad).

import rclpy
from rclpy.node import Node

from ros_paparazzi_interfaces.msg import Waypoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3

from ros_paparazzi_core.aux import geo_tools
from ros_paparazzi_core.data import gcs_data

import numpy as np


# For the filter
R2_IMU = 2.5e-4

or_x = gcs_data.origin[0]; or_y = gcs_data.origin[1]

class SIM_Kalman(Node):

    def __init__(self):
        super().__init__('SIM_Kalman')

        self.Kalman_publisher = self.create_publisher(Waypoint, 'waypoints/telemetry_gps', 10)
        # self.IMU_subscription = self.create_subscription(Vector3, 'sensors/imu', self.kalman_predict, 10)
        self.GPS_subscription = self.create_subscription(NavSatFix, 'sensors/gps', self.kalman_update, 10)
        self.GPS_subscription = self.create_subscription(Vector3, 'sensors/gps/speed', self.update_speed, 10)

        self.kalman_init()


    def kalman_init(self):

        dt = 0.3
        rho = R2_IMU*pow(dt, 4)

        self.A = np.matrix([[1, 0, dt,  0],
              [0, 1,  0, dt],
              [0, 0,  1,  0],
              [0, 0,  0,  1]
        ])
        self.B = np.matrix([[ 0,  0],
              [ 0,  0],
              [dt,  0],
              [ 0, dt]
        ])
        self.C = np.eye(4)

        self.Q = np.eye(4)*rho
        self.R = np.eye(4)
        self.P = np.eye(4)*500

        self.X = np.zeros((4, 1))
        self.Y = np.array([[0.0], [0.0], [0.0], [0.0]])



    def kalman_predict(self, msg):
        # Calculo de X = Ax + Bu
        U = np.array([[msg.x], [msg.y]])
        tmp1 = np.matmul(self.A, self.X)
        tmp2 = np.matmul(self.B, U)

        self.X = tmp1 + tmp2

        # Calculo de P = APA + Q
        tmp1 = np.matmul(self.A, self.P)
        tmp2 = np.matmul(tmp1, self.A.T)
        self.P = tmp2 + self.Q


    def kalman_update(self, msg):
        # Calculo de K = PC(CPC + R)^-1
        tmp1 = np.matmul(self.C, self.P)
        tmp2 = np.matmul(tmp1, self.C.T)
        tmp3 = tmp2 + self.R
        tmp4 = np.linalg.inv(tmp3)

        tmp1 = np.matmul(self.P, self.C.T)
        self.K = np.matmul(tmp1, tmp4)

        # Calculo de x = x + K(Y - Cx)
        x, y = geo_tools.wgs84_to_ltp(or_x, or_y, msg.latitude, msg.longitude)
        self.Y[0] = float(x)
        self.Y[1] = float(y)
        tmp1 = np.matmul(self.C, self.X)
        tmp2 = self.Y - tmp1
        tmp1 = np.matmul(self.K, tmp2)
        self.X = self.X + tmp1
        print(f'X = {self.X}')
        self.publish_state()

        # Calculo de P = (I - KC)P
        I = np.eye(self.P.shape[0])
        tmp1 = I - np.matmul(self.K, self.C)
        self.P = np.matmul(tmp1, self.P)
        print(f'K = {self.K}')


    def update_speed(self, msg):
        self.Y[2] = msg.x
        self.Y[3] = msg.y


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
