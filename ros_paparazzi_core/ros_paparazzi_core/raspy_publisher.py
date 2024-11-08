# This is the node that will run on the raspberry

import threading
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from ros_paparazzi_interfaces.msg import Waypoint

from ros_paparazzi_core.data import autopilot_data
from ros_paparazzi_core.paparazzi_receive import PPZI_TELEMETRY, TIME_THREAD
from ros_paparazzi_core.paparazzi_send import PPZI_DATALINK


# Variables globales
paparazzi_send = None
paparazzi_receive = None
time_thread = None


class Raspy_Publisher(Node):

    def __init__(self):
        super().__init__('Raspy_Publisher')
        self.publisher = self.create_publisher(NavSatFix, 'telemetry_gps', 10)
        self.suscriber = self.create_subscription(Waypoint, 'datalink_gps', self.datalink_callback, 10)

        # Crear un hilo para monitorear cambios en telemetry_data
        self.last_telemetry_data = []
        self.monitor_thread = threading.Thread(target=self.monitor_telemetry, daemon=True)
        self.monitor_thread.start()

        # Clase para mandar datos por el puerto serie (datalink)
        port = "/dev/ttyUSB0"
        self.paparazzi_send = PPZI_DATALINK(port)
        self.paparazzi_send.run()
        

    # Funcion que manda los mensajes datalink por el puerto serie cuando los recibe del topic
    def datalink_callback(self, msg):

        lat = msg.position.latitude
        lon = msg.position.longitude
        alt = msg.position.altitude
        wp_id = msg.wp_id

        self.get_logger().info(f'Receiving data: [{lat}, {lon}]')
        autopilot_data.waypoint_data.update(lat, lon, alt, wp_id)
        self.paparazzi_send.send()


    # Funcion que publica en el topic los mensajes telemetry que recibe por el puerto serie
    def telemetry_callback(self, data):
        # data = [float(i) for i in data] # Hay que convertir los datos a Float64, que es lo que acepta NatSat
        msg = NavSatFix()
        msg.header.stamp.sec = int(autopilot_data.tiempo)
        msg.header.stamp.nanosec = int(1e+9*(autopilot_data.tiempo - int(autopilot_data.tiempo)))
        msg.longitude = float(data.longitude*1e-07)
        msg.latitude = float(data.latitude*1e-07)
        msg.altitude = float(data.altitude/1000.0)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing data: [{msg.latitude}, {msg.longitude}, {msg.altitude}]')


    # Hilo para monitorear cambios en telemetry_data
    def monitor_telemetry(self):
        
        while True:
                
            current_data = autopilot_data.telemetry_data.recover()
                           
            if not self.last_telemetry_data:
                self.last_telemetry_data = current_data

            if current_data != self.last_telemetry_data:
                self.last_telemetry_data = current_data
                self.telemetry_callback(autopilot_data.telemetry_data)

            time.sleep(0.1)



def main(args=None):

    global paparazzi_send, paparazzi_receive, time_thread
    rclpy.init(args=args)

    port = "/dev/ttyUSB0"

    # Lanza los hilos
    paparazzi_time = TIME_THREAD()
    time_thread = threading.Thread(target=paparazzi_time.run, args=(autopilot_data,))
    time_thread.start()

    paparazzi_receive = PPZI_TELEMETRY(port)
    receive_thread = threading.Thread(target=paparazzi_receive.run)
    receive_thread.start()

    # Lanza el nodo ROS
    raspy_ros_node = Raspy_Publisher()
    rclpy.spin(raspy_ros_node)

    raspy_ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()