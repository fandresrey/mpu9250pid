#include "project.h"
#include "MPU9250.h"
#include "stdio.h"
#include <math.h>


// Constantes para el filtro complementario
#define ALPHA_ACC 98    // Factor de suavizado para el acelerómetro (0.98 * 100)
#define ALPHA_GYRO 98   // Factor de suavizado para el giroscopio (0.98 * 100)

// Constantes para el filtro de media móvil
#define WINDOW_SIZE 50   // Tamaño de la ventana para el filtro de media móvil

#define KP 2   // Ganancia Proporcional
#define KI 0   // Ganancia Integral
#define KD 0.1   // Ganancia Derivativa
float setpoint = 0.0;  // Valor deseado (setpoint)
float integral = 0.0;
float previous_error = 0.0;
unsigned long previous_time = 0;
int PID(float current_value, float dt) {
        char message[50];       // Buffer para el mensaje UART
    float error = setpoint - current_value;
    integral += error * dt/1000.0;
    float derivative = (error - previous_error) / (dt/1000.0);
    float output = KP * error + KI * integral + KD * derivative;

    previous_error = error;
    return output;
}
int main(void)
{
    CyGlobalIntEnable; /* Habilitar interrupciones globales */
    
    // Iniciar componente UART
    UART_Debug_Start();
    millis_Start();
    
    UART_Debug_PutString("**************\r\n");
    UART_Debug_PutString("    MPU9250   \r\n");
    UART_Debug_PutString("**************\r\n");
    
    // Iniciar componente I2C
    I2C_MPU9250_Master_Start();
    
    CyDelay(1000);  // Esperar un segundo
    
    char message[50];       // Buffer para el mensaje UART
    uint8_t connection = 0; // Variable para el estado de conexión
    
    // Escanear bus I2C y buscar dispositivos
    for (int address = 0; address < 128; address++) {
        if (I2C_MPU9250_Master_MasterSendStart(address, 0) == I2C_MPU9250_Master_MSTR_NO_ERROR) {
            sprintf(message, "Dispositivo encontrado en: 0x%02x\r\n", address);
            UART_Debug_PutString(message);
        }
        I2C_MPU9250_Master_MasterSendStop();
    }
    
    // Esperar hasta que MPU9250 esté conectado
    do {
        connection = MPU9250_IsConnected();
    } while (connection == 0);
    
    // Mostrar estado de conexión
    Connection_Led_Write(1);  // Encender LED de conexión
     

    // Iniciar MPU9250 y configurar escala del acelerómetro
    MPU9250_Start();
    MPU9250_SetAccFS(MPU9250_Acc_FS_2g);
    MPU9250_SetGyroFS(MPU9250_Gyro_FS_250);
    
    // Leer registro WHO AM I y comparar con el valor esperado
    uint8_t whoami = MPU9250_ReadWhoAmI();
    sprintf(message, "WHO AM I: 0x%02x - Esperado: 0x%02x\r\n", whoami, MPU9250_WHO_AM_I);
    UART_Debug_PutString(message);
    
    // Arreglo estático para almacenar datos crudos del sensor
    static uint8_t data[14];
    
    // Variables para los valores crudos y filtrados del sensor
    int gx,gy;
    int16_t ax_raw, gx_raw, ay_raw, az_raw, gy_raw,gz_raw;  // Valores crudos del acelerómetro y giroscopio
    int32_t ax_filtered_comp = 0;  // Valor filtrado con complementario
    int32_t ax_filtered_mm = 0;    // Valor filtrado con media móvil
    int32_t ax_buffer[WINDOW_SIZE] = {0};  // Buffer para el filtro de media móvil
    uint8_t ax_index = 0;  // Índice actual en el buffer
    int16_t pitch_angle_int, roll_angle_int;
    int32_t coun,dt,tiempo_prev;
    float girosc_ang_x_prev, girosc_ang_y_prev,girosc_ang_x,girosc_ang_y;
    float ang_x, ang_y;
    float ang_x_prev, ang_y_prev;
    float a =0.98;
    float pid_output = 0.0;
    for(;;)
    {
        CyDelay(10); // Esperar 10 ms
     
        // Leer datos crudos del acelerómetro y giroscopio
        MPU9250_ReadAccGyroRaw(&data[1]);
        
        // Convertir datos crudos a valores de 16 bits
        ax_raw = (int16_t)((data[1] << 8) | (data[2]& 0xFF ));
        ay_raw = (int16_t)((data[3] << 8) | (data[4]& 0xFF ));
        az_raw = (int16_t)((data[5] << 8) | (data[6] & 0xFF));
        gx_raw = (int16_t)((data[7] << 8) | (data[8] & 0xFF));
        gy_raw = (int16_t)((data[9] << 8) | (data[10]& 0xFF));
        gz_raw = (int16_t)((data[11] << 8) | (data[12] & 0xFF));
        
        dt = millis_ReadCounter()-tiempo_prev;
        tiempo_prev=millis_ReadCounter();

  
  
        float accel_ang_x = atan2f((float)ay_raw, sqrtf((float)ax_raw * ax_raw + (float)az_raw * az_raw)) * 180.0 / M_PI;
        float accel_ang_y = atan2f((float)-ax_raw, sqrtf((float)ay_raw * ay_raw + (float)az_raw * az_raw)) * 180.0 / M_PI;

        
        ang_x = a*(ang_x_prev+(gx_raw/131)*dt/1000.0) + (1-a)*accel_ang_x;
        ang_y = a*(ang_y_prev+(gy_raw/131)*dt/1000.0) + (1-a)*accel_ang_y;
        ang_x_prev=ang_x;
        ang_y_prev=ang_y;
        pitch_angle_int = (int32_t)ang_x; // Convertir a entero
        roll_angle_int = (int32_t)ang_y; // Convertir a entero
          
             pid_output = PID(ang_x, dt);
           int pid =(int)pid_output;
        sprintf(message, "Pitch: %d, pid: %d \n", pitch_angle_int, pid);
       // sprintf(message, "rawx: %d, rawy: %d, zero: %d ,Pitchg: %d, rowg: %d\n", gx_raw,gy_raw, 2,gx,gy);

       
        UART_Debug_PutString(message);
    }
}

