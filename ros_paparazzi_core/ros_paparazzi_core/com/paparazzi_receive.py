import serial
import time
import math
import time
import threading
import struct
# import logging

from ros_paparazzi_core.data import autopilot_data


# CONSTANTES
PPZ_START_BYTE = 0x50  # "P"
COM_START_BYTE = 0x52  # "R"
PPZ_SONAR_BYTE = 0x53  # "S"
PPZ_TELEMETRY_BYTE = 0x54  # "T"
PPZ_PERIODIC_BYTE = 0X42  # "B"
PPZ_HOME_BYTE = 0x48 # "H"
PPZ_IMU_BYTE = 0x49  # "I"
PPZ_MEASURE_BYTE = 0x4D  # "M"
PPZ_GPS_BYTE = 0x47  # "G"
PPZ_LIDAR_BYTE = 0x4C; # "L"
COM_FINAL_BYTE = 0x46  # "F"
COM_ANSWER_BYTE = 0x4F  # "O"
COM_NO_MEASURE_BYTE = 0x4E  # "N"
COM_LOSS_BYTE = 0x4C  # "L"
COM_HALL_BYTE = 0x48  # "H"
COM_VUELTA_BYTE = 0x56  # "V"
POSITIVO = 0x00  # Positive sign
NEGATIVO = 0x01  # Negative sign

RECEIVE_INTERVAL = 1e-02  # Intervalo en el que comprueba si llega un mensaje


# Función para pasar enteros a hexadecimal
def itoh(value, nbytes):
    nmax = math.pow(2, nbytes * 8) - 1
    if abs(value) > nmax:
        return
    hex_bytes = [0] * nbytes 
    for i in range(nbytes):
        hex_bytes[i] = (value & (0xff << (nbytes - 1 - i) * 8)) >> (8 * (nbytes - 1 - i))
    return hex_bytes

# Función para pasar bytes a enteros
def serial_byteToint(byte_array, length):
    num = 0
    i = length - 1
    while i >= 0:
        num |= byte_array[i] << (8 * i)
        i -= 1
    return num


# Funcion para pasar bytes a floats
def serial_byteTofloat(byte_array):

    byte_array = byte_array[:4]
    return struct.unpack('<f', bytes(byte_array))[0]
  

# Función para comprobar checksum
def compare_checksum(message, check, l_message):
    suma = 0
    for i in range(l_message - 2):
        suma += int(message[i])

    if suma == check:
        return True
    else:
        return False

# Función para ver el signo en los bytes con signo
def calculate_signo(signo):
    if signo == POSITIVO:
        return 1
    elif signo == NEGATIVO:
        return -1
    else:
        raise ValueError("Invalid sign value. Expected 0 (POSITIVE) or 1 (NEGATIVE).")



# ---------------------- HILOS ----------------------

 
# Hilo contador de tiempo
class TIME_THREAD(threading.Thread):
    def __init__(self, logger):
        super().__init__()
        self.interval = 0.05  # Intervalo en segundos
        self.running = True   # Controla el estado del hilo
        self.logger = logger

    def run(self, autopilot_data):
        while self.running:
            time.sleep(self.interval)
            autopilot_data.tiempo += self.interval
            self.logger.debug(f"Tiempo: {autopilot_data.tiempo:.2f} s")  # Para verificar el incremento

    def stop(self):
        self.running = False

    

# Hilo de comunicación con Papparazzi
class PPZI_TELEMETRY(threading.Thread):

    def __init__(self, logger, port='/dev/ttyUSB0'):
        super().__init__()
        self.port = port
        self.baud_rate = 115200
        self.ser = None

        # logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        # self.logger = logging.getLogger(__name__)
        self.logger = logger

    def run(self):
        # Setup conexion serie
        fd = serial.Serial(self.port, self.baud_rate, timeout=1)
            
        while True:
            data_av = fd.in_waiting
  
            if data_av > 0:
                self.logger.debug(f"[PPZI_RECEIVE] - Número bytes: {data_av}")
                # Todo esto es para convertir los datos leidos al formato 
                # que se usaba en el codigo en C
                datappzz = fd.read(data_av).strip().hex()
                datappzz = [datappzz[i:i+2] for i in range(0, len(datappzz), 2)]
                datappzz = [int(byte, 16) for byte in datappzz]

                # self.logger.debug(f"[PPZI_RECEIVE] - Mensaje Completo: {datappzz}")
                
                # Parte el mensaje
                messages = []
                message = []
                for i, byte in enumerate(datappzz):
                    if byte == 10:  # Salto de Linea
                        messages.append(message)
                        message = []
                    else:
                        message.append(byte)
                        if i == (len(datappzz) - 1):
                            messages.append(message)
                            message = []


                for message in messages:

                    # Ignorar mensajes vacíos
                    if len(message) == 0:
                        continue

                    # Añadido el len porque daba un error si solo llegaba un byte
                    if len(message) < 2 or message[0] != PPZ_START_BYTE:
                        self.logger.debug("[PPZI_RECEIVE] - NO EMPIEZA POR P \n")
                        time.sleep(RECEIVE_INTERVAL)
                        continue
                    

                    if message[1] == PPZ_TELEMETRY_BYTE:
                        if len(message) != 26:
                            self.logger.debug(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 26, Received {len(message)}")
                            time.sleep(RECEIVE_INTERVAL)
                            continue

                        hex_checksumppzz = [message[25], message[24]]
                        checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                        # TODO: Solve checksum issue
                        # if not compare_checksum(message, checksumppzz, message):
                        #     self.logger.info("[PPZI_RECEIVE] - Checksum TELEMETRY erróneo.")
                        #     time.sleep(RECEIVE_INTERVAL)
                        #     continue

                        sign_long = message[4]
                        hex_long = [message[8], message[7], message[6], message[5]]
                        longitud = calculate_signo(sign_long) * serial_byteToint(hex_long, 4)

                        sign_lat = message[9]
                        hex_lat = [message[13], message[12], message[11], message[10]]
                        latitud = calculate_signo(sign_lat) * serial_byteToint(hex_lat, 4)

                        # Temporalmente voy a quitar la altitud
                        sign_alt = datappzz[14]
                        if sign_alt != 0 and sign_alt != 1:
                            sign_alt = 0
                        hex_alt = [datappzz[18], datappzz[17], datappzz[16], datappzz[15]]    # Esta por ahora es la actitud
                        altitud = calculate_signo(sign_alt) * serial_byteToint(hex_alt, 4)
                        # altitud = 650000

                        hex_d_sonar = [message[22], message[21], message[20], message[19]]
                        d_sonar = serial_byteToint(hex_d_sonar, 4)

                        c_sonar = int(message[23])

                        autopilot_data.telemetry_data.update(autopilot_data.tiempo, longitud, latitud, altitud, 0)
                        self.logger.debug(f"([PPZI_RECEIVE] - Nuevo dato de telemetría: {autopilot_data.telemetry_data}")         

                        # with open(name_telemetria, "a") as telemetria:
                        #     telemetria.write(f"{time.time()} {datappzz[1]} {longitud} {latitud} {altitud} {d_sonar} {c_sonar}\n")

                    # Este no se esta usando
                    elif message[1] == PPZ_MEASURE_BYTE:
                        if len(message) != 26:
                            self.logger.debug(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 26, Received {len(message)}")
                            time.sleep(RECEIVE_INTERVAL)
                            continue

                        hex_checksumppzz = [message[25], message[24]]
                        checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                        if not compare_checksum(message, checksumppzz, data_av):
                            self.logger.info("[PPZI_RECEIVE] - Checksum MEASURE erróneo.")
                            time.sleep(RECEIVE_INTERVAL)
                            continue


                        sign_long = message[4]
                        hex_long = [message[8], message[7], message[6], message[5]]
                        longitud = calculate_signo(sign_long) * serial_byteToint(hex_long, 4)

                        sign_lat = message[9]
                        hex_lat = [message[13], message[12], message[11], message[10]]
                        latitud = calculate_signo(sign_lat) * serial_byteToint(hex_lat, 4)

                        sign_alt = message[14]
                        hex_alt = [message[17], message[16], message[15]]
                        altitud = calculate_signo(sign_alt) * serial_byteToint(hex_alt, 3)

                        hex_d_sonar = [message[21], message[20], message[19], message[18]]
                        d_sonar = serial_byteToint(hex_d_sonar, 4)

                        c_sonar = int(message[22])
                        intervalo = int(message[23])

                        # with open(name_telemetria, "a") as telemetria:
                        #     telemetria.write(f"{autopilot_data.tiempo} {datappzz[1]} {longitud} {latitud} {altitud} {d_sonar} {c_sonar}\n")

                        # with open(name_puntos_medida, "a") as puntos_medida:
                        #     puntos_medida.write(f"{autopilot_data.tiempo} {datappzz[1]} {longitud} {latitud} {altitud} {d_sonar} {c_sonar}\n")

                    # Este no se esta usando
                    elif message[1] == PPZ_SONAR_BYTE:
                        if len(message) != 6:
                            self.logger.debug(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 6, Received {len(message)}")
                            time.sleep(RECEIVE_INTERVAL)
                            continue

                        hex_checksumppzz = [message[5], message[4]]
                        checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                        # TODO: Solve checksum issue
                        # if not compare_checksum(message, checksumppzz, data_av):
                        #     self.logger.info("Checksum SONAR erróneo.")
                        #     time.sleep(RECEIVE_INTERVAL)
                        #     continue

                        self.logger.debug(f"[PPZI_RECEIVE] - Mensaje recibido correctamente por el autopiloto")
                        self.logger.debug(f"[PPZI_RECEIVE] - Nuevo dato de la sonda -- {message[2:3]}")

                        # MENSAJE RESPUESTA A SOLICITUD DE PROFUNDIDAD (esto no se si hace algo con ello)
                        # profundidad = int(datappzz[4])
                        # checksum = COM_START_BYTE + PPZ_START_BYTE + int(autopilot_data.tiempo) + profundidad
                        # hex_tiempo = itoh(int(autopilot_data.tiempo), 2)
                        # hex_profundidad = itoh(profundidad, 4)
                        # hex_checksum = itoh(checksum, 2)
                        
                        # fd.write(bytes([COM_START_BYTE, PPZ_START_BYTE, *hex_tiempo, *hex_profundidad, *hex_checksum]))


                    elif message[1] == PPZ_HOME_BYTE:
                        if len(message) != 20:
                            self.logger.debug(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 20, Received {len(message)}")
                            time.sleep(RECEIVE_INTERVAL)
                            continue

                        hex_checksumppzz = [message[19], message[18]]
                        checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                        # TODO: Solve checksum issue
                        # if not compare_checksum(message, checksumppzz, data_av):
                        #     self.logger.info("[PPZI_RECEIVE] - Checksum HOME erróneo.")
                        #     time.sleep(RECEIVE_INTERVAL)
                        #     continue


                        sign_long = message[4]
                        hex_long = [message[8], message[7], message[6], message[5]]
                        longitud = calculate_signo(sign_long) * serial_byteToint(hex_long, 4)

                        sign_lat = message[9]
                        hex_lat = [message[13], message[12], message[11], message[10]]
                        latitud = calculate_signo(sign_lat) * serial_byteToint(hex_lat, 4)

                        sign_alt = message[14]
                        hex_alt = [message[17], message[16], message[15]]
                        # altitud = calculate_signo(sign_alt) * serial_byteToint(hex_alt, 3)
                        altitud = 650000      # Hay algún problema con la altitud

                        autopilot_data.home_data.update(autopilot_data.tiempo, longitud, latitud, altitud, 1)
                        self.logger.debug(f"([PPZI_RECEIVE] - Nueva posición de HOME: {autopilot_data.home_data}")


                    elif message[1] == PPZ_IMU_BYTE:
                        if len(message) != 21:
                            self.logger.debug(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 21, Received {len(message)}")
                            time.sleep(RECEIVE_INTERVAL)
                            continue

                        hex_checksumppzz = [message[20], message[19]]
                        checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                        # TODO: Solve checksum issue
                        # if not compare_checksum(message, checksumppzz, data_av):
                        #     self.logger.info("[PPZI_RECEIVE] - Checksum IMU erróneo.")
                        #     time.sleep(RECEIVE_INTERVAL)
                        #     continue

                        sign_x = message[4]
                        hex_x = [message[8], message[7], message[6], message[5]]
                        accel_x = calculate_signo(sign_x) * serial_byteToint(hex_x, 4)

                        sign_y = message[9]
                        hex_y = [message[13], message[12], message[11], message[10]]
                        accel_y = calculate_signo(sign_y) * serial_byteToint(hex_y, 4)

                        sign_z = message[14]
                        hex_z = [message[18], message[17], message[16], message[15]]
                        accel_z = calculate_signo(sign_z) * serial_byteToint(hex_z, 3)

                        autopilot_data.imu_data.update(autopilot_data.tiempo, accel_x, accel_y, accel_z)
                        self.logger.debug(f"([PPZI_RECEIVE] - {autopilot_data.imu_data}")

                    
                    elif message[1] == PPZ_GPS_BYTE:
                        if len(message) != 20:
                            self.logger.debug(f"[PPZG_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 20, Received {len(message)}")
                            time.sleep(RECEIVE_INTERVAL)
                            continue

                        hex_checksumppzz = [message[19], message[18]]
                        checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                        # TODO: Solve checksum issue
                        # if not compare_checksum(message, checksumppzz, data_av):
                        #     self.logger.debug("[PPZG_RECEIVE] - Checksum GPS erróneo.")
                        #     time.sleep(RECEIVE_INTERVAL)
                        #     continue

                        sign = message[4]
                        hex_lon = [message[8], message[7], message[6], message[5]]
                        gps_lon = calculate_signo(sign)*serial_byteToint(hex_lon, 4)

                        sign = message[9]
                        hex_lat = [message[13], message[12], message[11], message[10]]
                        gps_lat = calculate_signo(sign)*serial_byteToint(hex_lat, 4)

                        hex_alt = [message[17], message[16], message[15], message[14]]
                        gps_alt = serial_byteToint(hex_alt, 4)

                        autopilot_data.gps_data.update(gps_lat, gps_lon, gps_alt)
                        self.logger.debug(f"[PPZG_RECEIVE] - GPS Data: Lat:{gps_lat}, Lon:{gps_lon}, Alt:{gps_alt}")


                    elif message[1] == PPZ_LIDAR_BYTE:
                        if len(message) != 14:
                            self.logger.debug(f"[PPZG_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 14, Received {len(message)}")
                            time.sleep(RECEIVE_INTERVAL)
                            continue

                        hex_checksumppzz = [message[13], message[12]]
                        checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                        # TODO: Solve checksum issue
                        # if not compare_checksum(message, checksumppzz, data_av):
                        #     self.logger.info("[PPZG_RECEIVE] - Checksum LIDAR erróneo.")
                        #     time.sleep(RECEIVE_INTERVAL)
                        #     continue

                        lidar_dist = [message[4], message[5], message[6], message[7]]
                        lidar_dist = serial_byteTofloat(lidar_dist)

                        lidar_ang = [message[8], message[9], message[10], message[11]]
                        lidar_ang = serial_byteTofloat(lidar_ang)

                        autopilot_data.lidar_data.update(lidar_dist, lidar_ang)
                        self.logger.debug(f"[PPZG_RECEIVE] - LiDaR Data: Distancia = {lidar_dist}")

                    else:
                        self.logger.debug(f"[PPZG_RECEIVE] - Message not recognized")


            time.sleep(RECEIVE_INTERVAL)

