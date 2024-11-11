import serial
import time
import math
import time
import threading

from ros_paparazzi_core.data import autopilot_data


# CONSTANTES
PPZ_START_BYTE = 0x50  # "P"
COM_START_BYTE = 0x52  # "R"
PPZ_SONAR_BYTE = 0x53  # "S"
PPZ_TELEMETRY_BYTE = 0x54  # "T"
PPZ_HOME_BYTE = 0x48; # "H"
PPZ_MEASURE_BYTE = 0x4D  # "M"
COM_FINAL_BYTE = 0x46  # "F"
COM_ANSWER_BYTE = 0x4F  # "O"
COM_NO_MEASURE_BYTE = 0x4E  # "N"
COM_LOSS_BYTE = 0x4C  # "L"
COM_HALL_BYTE = 0x48  # "H"
COM_VUELTA_BYTE = 0x56  # "V"
POSITIVO = 0x00  # Positive sign
NEGATIVO = 0x01  # Negative sign


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
    def __init__(self):
        super().__init__()
        self.interval = 0.05  # Intervalo en segundos
        self.running = True   # Controla el estado del hilo

    def run(self, autopilot_data):
        while self.running:
            time.sleep(self.interval)
            autopilot_data.tiempo += self.interval
            # print(f"Tiempo: {autopilot_data.tiempo:.2f} s")  # Para verificar el incremento

    def stop(self):
        self.running = False

    

# Hilo de comunicación con Papparazzi
class PPZI_TELEMETRY(threading.Thread):

    def __init__(self, port='/dev/ttyUSB0'):
        super().__init__()
        self.port = port
        self.baud_rate = 9600
        self.ser = None

    def run(self):
        # Setup conexion serie
        fd = serial.Serial(self.port, 9600, timeout=1)
        # fd = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
            
        while True:
            data_av = fd.in_waiting
            print(f"\n[PPZI_RECEIVE] - Número bytes: {data_av}")
            
            if data_av > 0:
                # Todo esto es para convertir los datos leidos al formato 
                # que se usaba en el codigo en C
                datappzz = fd.read(data_av).strip().hex()
                datappzz = [datappzz[i:i+2] for i in range(0, len(datappzz), 2)]
                datappzz = [int(byte, 16) for byte in datappzz]

                # Añadido el len porque daba un error si solo llegaba un byte
                if datappzz[0] != PPZ_START_BYTE or len(datappzz) < 2:
                    print("[PPZI_RECEIVE] - NO EMPIEZA POR P \n")
                    time.sleep(1)
                    continue
                

                if datappzz[1] == PPZ_TELEMETRY_BYTE:
                    if len(datappzz) != 25:
                        print(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 25, Received {len(datappzz)}")
                        time.sleep(1)
                        continue

                    hex_checksumppzz = [datappzz[24], datappzz[23]]
                    checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                    if not compare_checksum(datappzz, checksumppzz, data_av):
                        print("[PPZI_RECEIVE] - Checksum erróneo.")
                        time.sleep(1)
                        continue

                    # print("TELEMETRIA")

                    sign_long = datappzz[4]
                    hex_long = [datappzz[8], datappzz[7], datappzz[6], datappzz[5]]
                    longitud = calculate_signo(sign_long) * serial_byteToint(hex_long, 4)

                    sign_lat = datappzz[9]
                    hex_lat = [datappzz[13], datappzz[12], datappzz[11], datappzz[10]]
                    latitud = calculate_signo(sign_lat) * serial_byteToint(hex_lat, 4)

                    sign_alt = datappzz[14]
                    hex_alt = [datappzz[17], datappzz[16], datappzz[15]]
                    altitud = calculate_signo(sign_alt) * serial_byteToint(hex_alt, 3)

                    hex_d_sonar = [datappzz[21], datappzz[20], datappzz[19], datappzz[18]]
                    d_sonar = serial_byteToint(hex_d_sonar, 4)

                    c_sonar = int(datappzz[22])

                    # print(f"Longitud: {longitud}, Latitud: {latitud}, Altitud: {altitud}, D_Sonar: {d_sonar}, C_Sonar: {c_sonar}, Checksum: {checksumppzz}")
                    # autopilot_data.telemetry_data = [longitud, latitud, altitud, d_sonar, c_sonar]  # TERMINAR
                    autopilot_data.telemetry_data.update(autopilot_data.tiempo, longitud, latitud, altitud)
                    print(f"([PPZI_RECEIVE] - Nuevo dato de telemetría: {autopilot_data.telemetry_data}")         

                    # with open(name_telemetria, "a") as telemetria:
                    #     telemetria.write(f"{time.time()} {datappzz[1]} {longitud} {latitud} {altitud} {d_sonar} {c_sonar}\n")

                # Este no se esta usando
                if datappzz[1] == PPZ_MEASURE_BYTE:
                    if len(datappzz) != 26:
                        print(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 26, Received {len(datappzz)}")
                        time.sleep(1)
                        continue

                    hex_checksumppzz = [datappzz[25], datappzz[24]]
                    checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                    if not compare_checksum(datappzz, checksumppzz, data_av):
                        print("[PPZI_RECEIVE] - Checksum erróneo.")
                        time.sleep(1)
                        continue

                    print("MEDIDA")

                    sign_long = datappzz[4]
                    hex_long = [datappzz[8], datappzz[7], datappzz[6], datappzz[5]]
                    longitud = calculate_signo(sign_long) * serial_byteToint(hex_long, 4)

                    sign_lat = datappzz[9]
                    hex_lat = [datappzz[13], datappzz[12], datappzz[11], datappzz[10]]
                    latitud = calculate_signo(sign_lat) * serial_byteToint(hex_lat, 4)

                    sign_alt = datappzz[14]
                    hex_alt = [datappzz[17], datappzz[16], datappzz[15]]
                    altitud = calculate_signo(sign_alt) * serial_byteToint(hex_alt, 3)

                    hex_d_sonar = [datappzz[21], datappzz[20], datappzz[19], datappzz[18]]
                    d_sonar = serial_byteToint(hex_d_sonar, 4)

                    c_sonar = int(datappzz[22])
                    intervalo = int(datappzz[23])

                    # with open(name_telemetria, "a") as telemetria:
                    #     telemetria.write(f"{autopilot_data.tiempo} {datappzz[1]} {longitud} {latitud} {altitud} {d_sonar} {c_sonar}\n")

                    # with open(name_puntos_medida, "a") as puntos_medida:
                    #     puntos_medida.write(f"{autopilot_data.tiempo} {datappzz[1]} {longitud} {latitud} {altitud} {d_sonar} {c_sonar}\n")

                # Este no se esta usando
                if datappzz[1] == PPZ_SONAR_BYTE:
                    print(datappzz)
                    if len(datappzz) != 6:
                        print(f"NÚMERO BYTES INCORRECTO --> Expected 6, Received {len(datappzz)}")
                        time.sleep(1)
                        continue

                    hex_checksumppzz = [datappzz[5], datappzz[4]]
                    checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                    if not compare_checksum(datappzz, checksumppzz, data_av):
                        print("Checksum erróneo.")
                        time.sleep(1)
                        continue

                    print(f"[PPZI_RECEIVE] - Mensaje recibido correctamente por el autopiloto")
                    print(f"[PPZI_RECEIVE] - Nuevo dato de la sonda -- {datappzz[2:3]}")

                    # MENSAJE RESPUESTA A SOLICITUD DE PROFUNDIDAD (esto no se si hace algo con ello)
                    # profundidad = int(datappzz[4])
                    # checksum = COM_START_BYTE + PPZ_START_BYTE + int(autopilot_data.tiempo) + profundidad
                    # hex_tiempo = itoh(int(autopilot_data.tiempo), 2)
                    # hex_profundidad = itoh(profundidad, 4)
                    # hex_checksum = itoh(checksum, 2)
                    
                    # fd.write(bytes([COM_START_BYTE, PPZ_START_BYTE, *hex_tiempo, *hex_profundidad, *hex_checksum]))


                if datappzz[1] == PPZ_HOME_BYTE:
                    if len(datappzz) != 20:
                        print(f"[PPZI_RECEIVE] - NÚMERO BYTES INCORRECTO --> Expected 20, Received {len(datappzz)}")
                        time.sleep(1)
                        continue

                    hex_checksumppzz = [datappzz[19], datappzz[18]]
                    checksumppzz = serial_byteToint(hex_checksumppzz, 2)

                    if not compare_checksum(datappzz, checksumppzz, data_av):
                        print("[PPZI_RECEIVE] - Checksum erróneo.")
                        time.sleep(1)
                        continue

                    # print("TELEMETRIA")

                    sign_long = datappzz[4]
                    hex_long = [datappzz[8], datappzz[7], datappzz[6], datappzz[5]]
                    longitud = calculate_signo(sign_long) * serial_byteToint(hex_long, 4)

                    sign_lat = datappzz[9]
                    hex_lat = [datappzz[13], datappzz[12], datappzz[11], datappzz[10]]
                    print(hex_lat)
                    latitud = calculate_signo(sign_lat) * serial_byteToint(hex_lat, 4)

                    sign_alt = datappzz[14]
                    hex_alt = [datappzz[17], datappzz[16], datappzz[15]]
                    # altitud = calculate_signo(sign_alt) * serial_byteToint(hex_alt, 3)
                    altitud = 650000      # Hay algún problema con la altitud

                    # print(f"Longitud: {longitud}, Latitud: {latitud}, Altitud: {altitud}, D_Sonar: {d_sonar}, C_Sonar: {c_sonar}, Checksum: {checksumppzz}")
                    # autopilot_data.telemetry_data = [longitud, latitud, altitud, d_sonar, c_sonar]  # TERMINAR
                    autopilot_data.home_data.update(autopilot_data.tiempo, longitud, latitud, altitud)
                    print(f"([PPZI_RECEIVE] - Nueva posición de HOME: {autopilot_data.home_data}")


            time.sleep(1)

