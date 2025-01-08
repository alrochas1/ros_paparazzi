import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix

from ros_paparazzi_core.aux import sim_functions
from ros_paparazzi_interfaces.msg import KalmanPredict 
from ros_paparazzi_interfaces.msg import KalmanUpdate

import os
import time

class SIM_CORE(Node):

    def __init__(self):
        super().__init__('SIM_CORE')

        self.IMU_publisher = self.create_publisher(Vector3, 'sensors/imu', 10)
        self.GPS_publisher = self.create_publisher(NavSatFix, 'sensors/gps', 10)
        self.KalmanPredict = self.create_publisher(KalmanPredict, 'kalman/predict', 10)
        self.KalmanUpdate = self.create_publisher(KalmanUpdate, 'kalman/update', 10)

        # Configuración
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_config.yaml')
        self.base_dir = sim_functions.load_config(self, config_path)
        
        self.get_events_data()
        self.get_imu_data()
        self.get_gps_data()
        self.get_attitude_data()

        self.publish_data()


    # Asi no va a publicar el ultimo dato (como hay unos 30000 datos, tampoco pasa nada)
    def publish_data(self):

        i = 0
        sleep_time = 0
        while i < len(self.sensor_events) - 1:
            while sleep_time == 0:

                timestamp, sensor_id = self.sensor_events[i]
                next_timestamp = self.sensor_events[i + 1][0]

                self.get_logger().info(f"Publishing event: {sensor_id} @ {timestamp}")

                if sensor_id == "imu":
                    self.publish_predict()
                elif sensor_id == "gps":
                    self.publish_update()
                elif sensor_id == "attitude":
                    self.update_attitude()

                sleep_time = next_timestamp - timestamp

                if  i >= len(self.sensor_events) - 2:
                    self.get_logger().info(f"Closing the simulator ... \n Reset the simulation to continue (Ctrl + C)")
                    break
                else:
                    if sleep_time == 0: # In case there are multiple data at the same time
                       i += 1

            self.get_logger().debug(f"Sleeping for {sleep_time} seconds")
            time.sleep(sleep_time)
            i += 1
            sleep_time = 0
                    



    ##############################################
    ############### PUBLISH FUNCTIONS ############
    ############################################## 

    def publish_predict(self):
        # if self.imu_index >= len(self.t_imu):
        #     self.get_logger().info("Rebooting IMU Simulator")
        #     self.imu_index = 0

        imu_msg = Vector3()
        imu_msg.x = float(self.ax[self.imu_index])
        imu_msg.y = float(self.ay[self.imu_index])
        imu_msg.z = float(self.az[self.imu_index])
        self.IMU_publisher.publish(imu_msg)

        kf = KalmanPredict()
        kf.imu.x = imu_msg.x; kf.imu.y = imu_msg.y; kf.imu.z = imu_msg.z
        kf.dt = self.t_imu[self.imu_index] - self.t_imu[self.imu_index-1]
        self.KalmanPredict.publish(kf)
        self.get_logger().info(f'Publishing IMU_Data [t={self.t_imu[self.imu_index]}]: [{imu_msg.x}, {imu_msg.y}, {imu_msg.z}]')

        self.imu_index += 1


    def publish_update(self):
        # if self.gps_index >= len(self.t_gps):
        #     self.get_logger().info("Rebooting GPS Simulator")
        #     self.gps_index = 0

        kf_msg = KalmanUpdate()
        msg = NavSatFix()
        kf_msg.gps_speed.x = float(self.vx[self.gps_index] * 1e-02)
        kf_msg.gps_speed.x = float(self.vy[self.gps_index] * 1e-02)
        kf_msg.gps_speed.z = float(self.vz[self.gps_index] * 1e-02)
        kf_msg.theta = float(self.theta)
        
        msg.latitude = float(self.lat[self.gps_index] * 1e-07)
        msg.longitude = float(self.lon[self.gps_index] * 1e-07)
        kf_msg.latitude = msg.latitude
        kf_msg.longitude = msg.longitude

        self.GPS_publisher.publish(msg)
        self.KalmanUpdate.publish(kf_msg)

        self.get_logger().info(f'Publishing GPS_Speed [t={self.t_gps[self.gps_index]}]: [{msg.latitude}, {msg.longitude}]')
        self.gps_index += 1


    def update_attitude(self):
        # if self.attitude_index >= len(self.t_attitude):
        #     self.get_logger().info("Rebooting Attitude Simulator")
        #     self.attitude_index = 0

        self.theta = float(self.theta_data[self.attitude_index])
        self.get_logger().info(f'Publishing Theta [t={self.t_attitude[self.attitude_index]}]: [{self.theta} rad]')
        self.attitude_index += 1        


    ##############################################
    ############### READING FUNCTIONS ############
    ############################################## 


    def get_events_data(self):

        gps_time = sim_functions.get_time_vector(os.path.join(self.base_dir, "gps_data.txt"))
        imu_time = sim_functions.get_time_vector(os.path.join(self.base_dir, "imu_data.txt"))
        attitude_time = sim_functions.get_time_vector(os.path.join(self.base_dir, "attitude_data.txt"))

        self.sensor_events = []
        self.sensor_events.extend([(time, 'gps') for time in gps_time])
        self.sensor_events.extend([(time, 'imu') for time in imu_time])
        self.sensor_events.extend([(time, 'attitude') for time in attitude_time])

        self.sensor_events.sort(key=lambda x: x[0]) 


    def get_imu_data(self):
        sim_functions.read_txt(self, os.path.join(self.base_dir, "imu_data.txt"))
        self.t_imu = sim_functions.get_column(self.data, 0)
        self.ax = sim_functions.get_column(self.data, 8)
        self.ay = sim_functions.get_column(self.data, 9)
        self.az = sim_functions.get_column(self.data, 10)
        self.imu_index = 0


    def get_gps_data(self):
        sim_functions.read_txt(self, os.path.join(self.base_dir, "gps_data.txt"))
        self.t_gps = sim_functions.get_column(self.data, 0)
        self.lat = sim_functions.get_column(self.data, 5)
        self.lon = sim_functions.get_column(self.data, 6)
        self.vx = sim_functions.get_column(self.data, 9)
        self.vy = sim_functions.get_column(self.data, 10)
        self.vz = sim_functions.get_column(self.data, 11)
        self.gps_index = 0
 

    def get_attitude_data(self):
        sim_functions.read_txt(self, os.path.join(self.base_dir, "attitude_data.txt"))
        self.t_attitude = sim_functions.get_column(self.data, 0)
        self.theta_data = sim_functions.get_column(self.data, 3) # rad

        self.attitude_index = 0
        self.theta = 0
            

def main(args=None):
    rclpy.init(args=args)

    node = SIM_CORE()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()