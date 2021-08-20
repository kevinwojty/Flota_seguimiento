#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"

#include "../include/RFID/rfid.h"
#include "../include/RFID/rc522.h"
#include "../include/SIM/sim.h"
#include "../include/common.h"



/*Variables externas*/
extern QueueHandle_t Cola_Resp_Tarj,queue_fot_wr;
extern xSemaphoreHandle Mutex_UART_LTE;

/*Variables globales*/
uint32_t RFID=0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Tarea encargada de tomar lecturas del modulo RFID
void LectRfid (void *pvParameter)
{
    uint32_t last_user_ID, actual_user_ID = 0;

	ESP_LOGI("RFID","Iniciando tarea RFID");
	while(1)
	{
		uint8_t* serial_no = rc522_get_tag();
		if(serial_no != NULL)
		{
			last_user_ID =(int)serial_no[3] | (int)serial_no[2] << 8 | (int)serial_no[1] << 16 | (int)serial_no[0] << 24;
			free(serial_no);
			ESP_LOGI("RFID", "Tarjeta recibida: %u" ,last_user_ID);
			
			if(RFID == last_user_ID)	// Deslogueo
			{
				ESP_LOGI("RFID","DESLOGUEO de tarjeta");
				actual_user_ID=0;
			    xSemaphoreTake(Mutex_UART_LTE, portMAX_DELAY);

				Change_NonTransparent();

				Turn_off_gpio_SIM(LEDV);
				Turn_off_gpio_SIM(LEDR);
				Turn_on_gpio_SIM(BUZZER);
				vTaskDelay(100/portTICK_RATE_MS);	
				Turn_off_gpio_SIM(BUZZER);
				vTaskDelay(100/portTICK_RATE_MS);	
				Turn_on_gpio_SIM(BUZZER);
				vTaskDelay(100/portTICK_RATE_MS);	
				Turn_off_gpio_SIM(BUZZER);
				Turn_on_gpio_SIM(LEDR);

				Change_Transparent();

				char temp[50],envio[80];
				ArmadoTrInicio(temp,TR_RFID);
				sprintf(envio,"%s,deslogueo,%s",temp,TR_FIN);
				Send_Transparent(envio,strlen(envio));

			    xSemaphoreGive(Mutex_UART_LTE);
				RFID=0;
				uint8_t data=4;	// es una foto de deslogueo
				xQueueSendToBack(queue_fot_wr,&data,portMAX_DELAY);
			} 			
			else	//caso tarjeta consulta
			{
			    xSemaphoreTake(Mutex_UART_LTE, portMAX_DELAY);   


				RFID=last_user_ID;
				char temp[50],envio[80];
				ArmadoTrInicio(temp,TR_RFID);
				sprintf(envio,"%s,logueo,%s",temp,TR_FIN);				
				Send_Transparent(envio,strlen(envio));
				RFID=0;
				xSemaphoreGive(Mutex_UART_LTE);  

					
				//Contestación de la base de datos
				char respuesta=0;

				xQueueReceive(Cola_Resp_Tarj,&respuesta,portMAX_DELAY);
			    xSemaphoreTake(Mutex_UART_LTE, portMAX_DELAY);     


				if(respuesta == 1)	//Tarjeta registrada
				{

					if(actual_user_ID != 0)	// no se deslogueo el anterior
					{
						RFID = actual_user_ID;
						ArmadoTrInicio(temp,TR_RFID);
						sprintf(envio,"%s,deslogueo,%s",temp,TR_FIN);				
						Send_Transparent(envio,strlen(envio));
					}

					Change_NonTransparent();
					RFID = last_user_ID;
					actual_user_ID = last_user_ID;
					Turn_off_gpio_SIM(LEDR);
					Turn_off_gpio_SIM(LEDV);
					Turn_on_gpio_SIM(BUZZER);
					vTaskDelay(300/portTICK_RATE_MS);	
					Turn_off_gpio_SIM(BUZZER);
					Turn_on_gpio_SIM(LEDV);
					uint8_t data=2;	// es una foto de logueo
					xQueueSendToBack(queue_fot_wr,&data,portMAX_DELAY);
				}

				else	//Tarjeta no registrada
				{
					Turn_off_gpio_SIM(LEDR);
					Turn_off_gpio_SIM(LEDV);
					Change_NonTransparent();
					Turn_on_gpio_SIM(BUZZER);
					vTaskDelay(100/portTICK_RATE_MS);	
					Turn_off_gpio_SIM(BUZZER);
					vTaskDelay(300/portTICK_RATE_MS);	
					Turn_on_gpio_SIM(BUZZER);
					vTaskDelay(100/portTICK_RATE_MS);	
					Turn_off_gpio_SIM(BUZZER);
					vTaskDelay(300/portTICK_RATE_MS);	
					Turn_on_gpio_SIM(BUZZER);
					vTaskDelay(100/portTICK_RATE_MS);	
					Turn_off_gpio_SIM(BUZZER);
					Turn_on_gpio_SIM(LEDR);
				}

				Change_Transparent();
			    xSemaphoreGive(Mutex_UART_LTE);
				
			}
			vTaskDelay(1500/portTICK_RATE_MS);	// Delay para evitar lecturas multiples de la misma tarjeta
		}
		vTaskDelay(100/portTICK_RATE_MS);
	}
	vTaskDelete(NULL);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
esp_err_t rc522_start(rc522_start_args_t start_args) {
    assert(rc522_spi_init(
        start_args.miso_io, 
        start_args.mosi_io, 
        start_args.sck_io, 
        start_args.cs) == ESP_OK);
    
    if(rc522_init() != ESP_OK)
	{
		return ESP_FAIL;
	}
    xTaskCreate(&LectRfid, "LectRfid", 3072,NULL,tskIDLE_PRIORITY+2,NULL);
    
	return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
esp_err_t rfid_start()
{
	const rc522_start_args_t start_args = {
	.miso_io  = MISO_V,
	.mosi_io  = MOSI_V,
	.sck_io   = SCK_V,
	.cs		  = CS_RFID,
	//.callback = &tag_handler
	};

    rc522_start(start_args);	//inicializo el RFID y creo la tarea correspondiente
	return ESP_OK;
}