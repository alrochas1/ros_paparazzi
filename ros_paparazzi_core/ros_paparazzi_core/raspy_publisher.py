# This is the node that will run on the raspberry

import threading
# import time

import rclpy
from rclpy.node import Node

from ros_paparazzi_interfaces.msg import Waypoint
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, LaserScan

from ros_paparazzi_core.data import autopilot_data
from ros_paparazzi_core.com.paparazzi_receive import PPZI_TELEMETRY, TIME_THREAD
from ros_paparazzi_core.com.paparazzi_send import PPZI_DATALINK


# Variables globales
paparazzi_send = None
paparazzi_receive = None
time_thread = None


SR_WAYPOINT = 0x57     # W
SR_HOME = 0X48         # H


# PORT = "/dev/ttyUSB0"
PORT = "/dev/serial0"


class Raspy_Publisher(Node):

    def __init__(self):
        super().__init__('Raspy_Publisher')
        self.telemetry_publisher = self.create_publisher(Waypoint, 'waypoints/telemetry_gps', 10)
        self.suscriber = self.create_subscription(Waypoint, 'waypoints/datalink', self.waypoint_callback, 10)
        self.IMU_publisher = self.create_publisher(Vector3, 'sensors/imu', 10)
        self.GPS_publisher = self.create_publisher(NavSatFix, 'sensors/gps', 10)
        self.LiDaR_publisher = self.create_publisher(LaserScan, 'sensors/lidar', 10)

        autopilot_data.telemetry_data.register_callback(self.telemetry_callback)
        autopilot_data.imu_data.register_callback(self.imu_callback)
        autopilot_data.gps_data.register_callback(self.gps_callback)
        autopilot_data.lidar_data.register_callback(self.lidar_callback)

        # self.create_monitor_threads()
        
        # Clase para mandar datos por el puerto serie (datalink)
        self.paparazzi_send = PPZI_DATALINK(PORT)
        self.paparazzi_send.run()



    # Funcion que manda los mensajes datalink por el puerto serie cuando los recibe del topic
    def waypoint_callback(self, msg):

        # Hay que pasarlo a entero, que es lo que entiende Paparazzi
        lat = int(msg.gps.latitude*1e+07)
        lon = int(msg.gps.longitude*1e+07)
        alt = int(msg.gps.altitude)
        wp_id = msg.wp_id

        self.get_logger().info(f'Receiving data [{wp_id}]: [{lat*1e-07:.7f}, {lon*1e-07:.7f}]')
        autopilot_data.waypoint_data.update(lat, lon, alt, wp_id)
        
        if wp_id == 0: msg_type = SR_HOME
        else: msg_type = SR_WAYPOINT
        self.paparazzi_send.send(msg_type)
        

    # --------------------------------- FUNCIONES DE MONITOREO ---------------------------------

    # Funcion que publica en el topic los mensajes telemetry que recibe por el puerto serie
    def telemetry_callback(self, data):
        msg = Waypoint()
        msg.gps.header.stamp.sec = int(autopilot_data.tiempo)
        msg.gps.header.stamp.nanosec = int(1e+9*(autopilot_data.tiempo - int(autopilot_data.tiempo)))
        msg.gps.longitude = float(data.longitude*1e-07)
        msg.gps.latitude = float(data.latitude*1e-07)
        msg.gps.altitude = float(data.altitude*1e-07)
        msg.wp_id = int(data.wp_id)
        self.telemetry_publisher.publish(msg)
        self.get_logger().info(f'Publishing Telemetry_Data[{msg.wp_id}]: [{msg.gps.latitude:.7f}, {msg.gps.longitude:.7f}, {msg.gps.altitude:.2f}]')


    def imu_callback(self, data):

        msg = Vector3()
        msg.x = float(data.x)
        msg.y = float(data.y)
        msg.z = float(data.z)

        self.IMU_publisher.publish(msg)
        self.get_logger().info(f'Publishing IMU_Data: [{msg.x}, {msg.y}, {msg.z}]')


    def gps_callback(self, data):

        msg = NavSatFix()
        msg.longitude = float(data.lon*1e-07)
        msg.latitude = float(data.lat*1e-07)
        msg.altitude = float(data.alt*1e-07)

        self.GPS_publisher.publish(msg)
        self.get_logger().info(f'Publishing GPS_Data: [{msg.latitude}, {msg.longitude}]')

    
    def lidar_callback(self, data):

        msg = LaserScan()
        msg.ranges = [float(data.distance)]
        msg.angle_increment = float(data.angle)
        msg.range_min = 0.1
        msg.range_max = 12.0

        self.LiDaR_publisher.publish(msg)
        self.get_logger().info(f'Publishing LIDAR_Data: {data.distance} m, Angle: {data.angle}º')
        



def main(args=None):

    global paparazzi_send, paparazzi_receive, time_thread
    rclpy.init(args=args)
    raspy_node = Raspy_Publisher()

    # Lanza los hilos
    paparazzi_time = TIME_THREAD(raspy_node.get_logger())
    time_thread = threading.Thread(target=paparazzi_time.run, args=(autopilot_data,))
    time_thread.start()

    paparazzi_receive = PPZI_TELEMETRY(raspy_node.get_logger(), PORT)
    receive_thread = threading.Thread(target=paparazzi_receive.run)
    receive_thread.start()

    # Lanza el nodo ROS
    rclpy.spin(raspy_node)

    raspy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
