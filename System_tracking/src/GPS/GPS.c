// UART driver
#include "driver/uart.h"

// Error library
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <string.h>

#include "../include/GPS/GPS.h"
#include "../include/common.h"
#include "../include/SIM/sim.h"

extern xSemaphoreHandle Mutex_UART_LTE;

void Init_GPS ()
{
    ESP_LOGI("GPS","Inicializacion UART0..\r\n\r\n");
	// Inicializacion de la UART0 (GPS en 9600)
	uart_config_t uart_config = 
	{
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_0_TX, UART_0_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);	

    xTaskCreate(Task_GPS, "Task_GPS", 2048, NULL, tskIDLE_PRIORITY + 2UL, NULL); 
}


void Task_GPS (void *pvParameter)
{
	uint8_t data_rd[100],len;
    uint16_t i = 0;
    //$GPRMC,044235.000,A,4322.0289,N,00824.5210,W,0.39,65.46,020615,,,A*44
    char* data = (char*) data_rd;
    bzero(data_rd,100);
    uint8_t temp,repeticiones=0;

    while (1) 
	{
		// Leo datos de la UART
        len = uart_read_bytes(UART_NUM_0, &temp, 1, 50/portTICK_RATE_MS);
        if(len>0)
        {
            data[i] = (char) temp;
            if(data[i] == '\n') //pregunto si termino la linea
            {
                const char GPRMC[]="GPRMC";

                ESP_LOGD("GPS","%s",data);

                if(strncmp(GPRMC, &data[1],strlen(GPRMC)) == 0) //Me fijo que sea de datos de gps
                {
                    char * actual = NULL, *sgte = NULL;
                    const char coma= ',';

                    actual = strchr(data,coma); //busco la primera coma
                    actual++;
                    sgte = strchr(actual,coma); //busco la segunda coma
                    actual=sgte+1;
                    sgte = strchr(actual,coma); //busco la tercera coma

                    if(*actual == 'A')  //me aseguro que la trama sea correcta
                    {
                        if(repeticiones > 8)
                        {
                            char inicio[50],envio[200];
                            ArmadoTrInicio(inicio,TR_POS_VEL);
                            sprintf(envio,"%s,%s,%s",inicio,data,TR_FIN);
                            xSemaphoreTake(Mutex_UART_LTE,portMAX_DELAY); 
                            Send_Transparent(envio,strlen(envio));           
                            xSemaphoreGive(Mutex_UART_LTE);
                            repeticiones = 0;
                        }
                        else
                            repeticiones++;
                    }
                }
                bzero(data_rd,strlen(data));            
                i=0;
            }  
            else  
                i++;
        }
    }
}