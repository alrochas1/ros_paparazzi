import serial
import struct
import time
# import random
import threading

from ros_paparazzi_core.data import autopilot_data

COM_START_BYTE = 0x52  # R
SR_WAYPOINT = 0x57     # W

def calculate_checksums(data):
    checksum = sum(data) & 0xFFFF   # 2 Bytes
    data.append(checksum & 0xFF)    
    data.append((checksum >> 8) & 0xFF)

# Esta seguramente no es necesaria
def print_message(message):
    print("Mensaje enviado: ", " ".join(f"{byte:02X}" for byte in message))


# ---------------------- HILOS ----------------------

class PPZI_DATALINK(threading.Thread):

    def __init__(self, port='/dev/ttyUSB0'):
        super().__init__()
        self.port = port
        self.baud_rate = 9600
        self.ser = None


    def run(self):
        # Configuración del puerto serie
        try:
            self.ser = serial.Serial(self.port, baudrate=self.baud_rate, timeout=1)
            time.sleep(1)
            print("Conexión establecida con el autopiloto.")
        except serial.SerialException:
            print("Error al abrir el puerto serie.")
            self.running = False


    def send(self):

        if not self.ser:
            print("Puerto serie no está disponible.")
            return

        length = 0x11  # 17 bytes
        msg_id = SR_WAYPOINT


        [lat, lon, alt, wp_id] = autopilot_data.waypoint_data.recover()
        print(f"([PPZI_SEND] - Nuevo envio de dato: {autopilot_data.waypoint_data}")
        
        # wp_id = 14
        # lat = int(40.4506399 * 1e7 + (random.randint(-10000, 10000)))
        # lon = int(-3.7260463 * 1e7 + (random.randint(-10000, 10000)))
        # alt = int(650.0 * 1e3)

        # Construcción del mensaje
        msg_data = struct.pack('<iii', lat, lon, alt)
        message = [COM_START_BYTE, msg_id, wp_id] + list(msg_data)

        # Calcular y añadir el checksum
        calculate_checksums(message)

        # Imprimir el mensaje
        # print_message(message)
        print(f"[PPZI_SEND] - Coordenadas enviadas: [{lat * 1e-7:.7f}, {lon * 1e-7:.7f}]")


        # Enviar el mensaje
        self.ser.write(bytearray(message))
        time.sleep(2)

    def close(self):

        self.ser.close()
        print("Conexión cerrada.")
