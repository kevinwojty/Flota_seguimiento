#include <stdio.h>
#include "driver/uart.h"

//------------------------------------------------------------------------------------
//  DEFINES
//------------------------------------------------------------------------------------

#define TX1 33
#define RX1 32

#define PWR_SIM 1 

#define UART_SIM UART_NUM_1
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define UART_SIM_VEL    921600
#define SIM_BUF_SIZE    2048

#define LEDV 3 //Numero de GPIO usado en la SIM
#define LEDR 6
#define LEDFLASH 43
#define BUZZER 41

#define ON  1
#define OFF 0

#define IP  "4"
#define PORT "18936"
#define IP_PORT "AT+CIPOPEN=0,\"TCP\",\"" IP ".tcp.ngrok.io\"," PORT "\r"
//"AT+CIPOPEN=0,\"TCP\",\"4.tcp.ngrok.io\",15708\r"

//------------------------------------------------------------------------------------
//  DECLARACIÓN DE FUNCIONES
//------------------------------------------------------------------------------------

void Init_UART_1(void);
void Open_Com_TCP(void);
void Close_Com_TCP(void);
void Set_NonTransparent(void);
void Send_NonTransparent(const char* data);
void End_NonTransparent(void);
void Set_Transparent(void);
void Send_Transparent(const char* data,uint32_t size);
void End_Transparent(void);
void Get_GPS (void);
void Analizar_Trama (char *dtmp);
void Init_LTE(uint8_t estado);
void Init_SIM(void);
void Config_RTC(void);
void Task_Send_GPS_LTE(void *pvParameters);
void Task_Send_Cam_LTE(void *pvParameters);
void Turn_on_gpio_SIM (uint8_t pin);
void Turn_off_gpio_SIM (uint8_t pin);
void Init_Input_Output_sim(void);
void ArmadoTrInicio(char* devolucion,char* TipoTrama);
void Change_Transparent(void);
void Change_NonTransparent(void);
void Task_Send_Basura(void *pvParameter);
void Task_Control_SIM(void *pvParameter);
void Estado_Dispositivo (uint8_t estado);
void reset_SIM();