# This is the node that will run on the raspberry

import threading
import time

import rclpy
from rclpy.node import Node

from ros_paparazzi_interfaces.msg import Waypoint
from geometry_msgs.msg import Vector3

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
        self.publisher = self.create_publisher(Waypoint, 'telemetry_gps', 10)   #TODO: Change the name of this 
        self.suscriber = self.create_subscription(Waypoint, 'waypoints/datalink', self.waypoint_callback, 10)
        self.IMU_publisher = self.create_publisher(Vector3, 'sensors/imu', 10)

        # Crear un hilo para monitorear cambios en telemetry_data
        self.last_telemetry_data = None
        self.last_home_data = None
        self.last_imu_data = None
        self.start_monitor_thread(autopilot_data.telemetry_data, self.telemetry_callback, self.last_telemetry_data)
        self.start_monitor_thread(autopilot_data.home_data, self.telemetry_callback, self.last_home_data)
        self.start_monitor_thread(autopilot_data.imu_data, self.imu_callback, self.last_imu_data)

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
        # data = [float(i) for i in data] # Hay que convertir los datos a Float64, que es lo que acepta NatSat
        msg = Waypoint()
        msg.gps.header.stamp.sec = int(autopilot_data.tiempo)
        msg.gps.header.stamp.nanosec = int(1e+9*(autopilot_data.tiempo - int(autopilot_data.tiempo)))
        msg.gps.longitude = float(data.longitude*1e-07)
        msg.gps.latitude = float(data.latitude*1e-07)
        msg.gps.altitude = float(data.altitude/1000.0)
        msg.wp_id = int(data.wp_id)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing Telemetry_Data[{msg.wp_id}]: [{msg.gps.latitude:.7f}, {msg.gps.longitude:.7f}, {msg.gps.altitude:.2f}]')


    def imu_callback(self, data):

        msg = Vector3()
        msg.x = data.x
        msg.y = data.y
        msg.z = data.z

        self.IMU_publisher.publish(msg)
        self.get_logger().info(f'Publishing IMU_Data: [{msg.x}, {msg.y}, {msg.z}]')
        



    # Función para iniciar un hilo de monitoreo genérico
    def start_monitor_thread(self, variable, callback, last_value):
        monitor_thread = threading.Thread(
            target=self.monitor_variable, args=(variable, callback, last_value), daemon=True
        )
        monitor_thread.start()


    # Función genérica para monitorear cambios en cualquier variable
    def monitor_variable(self, variable, callback, last_value):
        
        # last_value = self.last_telemetry_data

        while True:
            current_value = variable.recover()
            
            if last_value is None:
                last_value = current_value

            if current_value != last_value:
                last_value = current_value
                callback(variable)

            time.sleep(0.1)

    


def main(args=None):

    global paparazzi_send, paparazzi_receive, time_thread
    rclpy.init(args=args)

    port = "/dev/serial0"

    # Lanza los hilos
    paparazzi_time = TIME_THREAD()
    time_thread = threading.Thread(target=paparazzi_time.run, args=(autopilot_data,))
    time_thread.start()

    paparazzi_receive = PPZI_TELEMETRY(PORT)
    receive_thread = threading.Thread(target=paparazzi_receive.run)
    receive_thread.start()

    # Lanza el nodo ROS
    raspy_ros_node = Raspy_Publisher()
    rclpy.spin(raspy_ros_node)

    raspy_ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
