#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <wiringSerial.h>

#include <ros_paparazzi_cpp/com/paparazzi_receive.h>


#define PPZ_START_BYTE 0x50 // "P"
#define COM_START_BYTE 0x52 // "R"

#define PPZ_SONAR_BYTE 0x53 // "S"
#define PPZ_TELEMETRY_BYTE 0x54 // "T"
#define PPZ_MEASURE_BYTE 0x4D // "M"
#define PPZ_IMU_BYTE 0x49 // "I"

#define COM_FINAL_BYTE 0x46 //"F"
#define COM_ANSWER_BYTE 0x4F //"O"
#define COM_NO_MEASURE_BYTE 0x4E //"N"
#define COM_LOSS_BYTE 0x4C // "L"
#define COM_HALL_BYTE 0x48 //"H"
#define COM_VUELTA_BYTE 0x56 // "V"

#define POSITIVO 0x00 
#define NEGATIVO 0x01


int fd, data_av, p, checksumppzz;
time_t t;
struct tm *tm;


PI_THREAD(comunicacion_ppzz) {
    for(;;) {
        data_av = serialDataAvail(fd);
        p = 0;
        printf("Número bytes: %i\n", data_av);

        if (data_av > 0) {
            char datappzz[data_av];
            while (p < data_av) {
                datappzz[p] = serialGetchar(fd);
                p++;
            }

            // Dividir los mensajes utilizando salto de línea
            char *message_start = datappzz;
            for (int i = 0; i < data_av; i++) {
                if (datappzz[i] == 10) {
                    datappzz[i] = '\0';
                    char *message = message_start;
                    
                    // Procesar este mensaje
                    if (message[0] != PPZ_START_BYTE) {
                        printf("NO EMPIEZA POR P\n");
                        continue;
                    }

                    // Tipos de mensaje
                    switch (message[1]){

                    case PPZ_TELEMETRY_BYTE:

                        if (i != 25) {
                            printf("NÚMERO BYTES INCORRECTO\n");
                            continue;
                        }


                        char hex_checksumppzz[2] = {datappzz[24],datappzz[23]};
                        checksumppzz = serial_byteToint(hex_checksumppzz,2);
                        
                        if(compare_checksum(datappzz, checksumppzz, data_av)==0){
                            printf("Checksum erróneo.\n");
                            delay(1000);
                            continue;
				        }

                        char sign_long = datappzz[4];
                        char hex_long[4]={datappzz[8],datappzz[7],datappzz[6],datappzz[5]};
                        long int longitud = calculate_signo(sign_long)*serial_byteToint(hex_long,4);
                        
                        char sign_lat = datappzz[9];
                        char hex_lat[4]={datappzz[13],datappzz[12],datappzz[11],datappzz[10]};
                        long int latitud = calculate_signo(sign_lat)*serial_byteToint(hex_lat,4);
                        
                        char sign_alt = datappzz[14];
                        char hex_alt[4]={datappzz[17],datappzz[16],datappzz[15]};
                        long int altitud = calculate_signo(sign_alt)*serial_byteToint(hex_alt,3);

                        printf("Longitud: %li, Latitud: %li, Altitud: %li\n",
						longitud, latitud, altitud);

                        GpsData gps_data = {0, longitud, latitud, altitud}; // Ejemplo
                        SensorData data = gps_data;
                        node_ros2->add_sensor_data(data);
                        
                        break;


                    case PPZ_IMU_BYTE:

                        ImuData imu_data = {-1.0, 1.0, 0.0};  // Ejemplo
                        SensorData data = imu_data;
                        node_ros2->add_sensor_data(data);
                    

                    // Add here your message
                    default:
                        break;
                    }
                

                    // Actualizamos el puntero para el siguiente mensaje
                    message_start = &datappzz[i+1];
                }
            }
        }
    }
}



int main(){
	 wiringPiSetup();
	 fd = serialOpen("/dev/serial0", 9600); //Puerto serie
	 
	 t=time(NULL);
	 tm=localtime(&t);
	 	 
	 //ARRANQUE HILOS
	 piThreadCreate(comunicacion_ppzz);

	 return 0;
 }


