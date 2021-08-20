/*  
------------------------------------------------------------------------------------
    UART
    
    Hardware:
    
------------------------------------------------------------------------------------
*/

//------------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h" 
#include "esp_intr_alloc.h" 

#include "../include/SIM/sim.h"
#include "../include/RFID/rfid.h"
#include "../include/camera/camera.h"
#include "../include/Acel/Acelerometro.h"
#include "../include/SD/sd.h"
#include "../include/GPS/GPS.h"
#include "../include/common.h"
#include "esp_camera.h"

//------------------------------------------------------------------------------------
//  VARIABLES GLOBALES
//------------------------------------------------------------------------------------
QueueHandle_t uart1_queue;
QueueHandle_t Cola_Resp_Tarj=NULL;

SemaphoreHandle_t Mutex_UART_LTE = NULL, Mutex_Trama_GPS = NULL,mutex_sd = NULL;
SemaphoreHandle_t sem_fin_foto = NULL ,sem_fot_dis = NULL, sem_take_pic= NULL;
SemaphoreHandle_t sem_sim_rdy = NULL, sem_sim_ok = NULL ;  
QueueHandle_t     queue_fot_send = NULL, queue_fot_wr = NULL,queue_rec_sd = NULL;


camera_fb_t* imagen[TAM_VEC_FOTOS];
uint8_t MAC[6];

//************************************************************************************
//                                      FUNCIONES
//************************************************************************************

//------------------------------------------------------------------------------------
// app_main
//      Funcion principal
//------------------------------------------------------------------------------------
void app_main()
{   
  uint8_t estado=0;  
	esp_read_mac(MAC,ESP_MAC_WIFI_STA);
	
	//Queues y semaforos camara
    sem_fin_foto = xSemaphoreCreateBinary();
    sem_fot_dis = xSemaphoreCreateBinary();
    sem_take_pic = xSemaphoreCreateBinary();//empiezan tomados los semaforos
    
    queue_fot_send = xQueueCreate(5,sizeof(uint8_t));
    queue_fot_wr = xQueueCreate(5,sizeof(uint8_t));


    //Semaforos SD

    queue_rec_sd = xQueueCreate(5,sizeof(uint8_t));
	  mutex_sd = xSemaphoreCreateMutex();
    xSemaphoreGive(mutex_sd);	// El mutex comienza liberado para que lo pueda tomar quien quiera
    

    //Queues y Semaforos del SIM
    Mutex_UART_LTE = xSemaphoreCreateMutex();
    sem_sim_rdy= xSemaphoreCreateBinary();
    sem_sim_ok  = xSemaphoreCreateBinary();

    //Chequeo    
    if(sem_sim_ok == NULL || sem_fin_foto == NULL || sem_fot_dis == NULL || sem_take_pic == NULL || mutex_sd == NULL) 
    {
      ESP_LOGE("ESP","ERROR AL CREAR LAS COLAS, el dispositivo se reiniciara");
      estado=1;   
    }
    if(Mutex_UART_LTE == NULL || queue_rec_sd == NULL || queue_fot_send == NULL || queue_fot_wr == NULL)
    {
      ESP_LOGE("ESP","ERROR AL CREAR LAS COLAS, el dispositivo se reiniciara");
      estado=1;         
    }

    //CAMARA Y SD

    if(camera_init_main() != ESP_OK)
    {
      estado=2;
    }
    if(sd_init_sdmode() != ESP_OK)
    {
      estado=3;
    }
    mytimer_init();
    create_sd_tasks();

    Init_SIM(); // Inicializacion de la UART de la SIM

    //RFID  Tiene que ir previo al acelerometro ya que es el que inicializa el SPI!

    //Queues y semaforos RFID
    Cola_Resp_Tarj = xQueueCreate(1,sizeof(char));
    
	  if(rfid_start() != ESP_OK)
    {
      estado=4;
    }

    //Acelerometro

    if(acelerometro_init() != ESP_OK)
    {
      estado=5;
    }

    //GPS externo
    #ifdef  GPS_CONECTADO
      Init_GPS();
    #endif

    xSemaphoreTake(sem_sim_rdy,60000/portTICK_PERIOD_MS); 

    Init_Input_Output_sim();    //Inicializacion de entradas y salidas SIM y ESP

    vTaskDelay(10000/portTICK_PERIOD_MS);
    Init_LTE(estado);         //Inicializacion del GPS y la comunicacion LTE

    xTaskCreate(Task_Send_Basura, "Task_Send_Basura", 2048, NULL, tskIDLE_PRIORITY + 1UL, NULL); 
    xSemaphoreGive(Mutex_UART_LTE); // El mutex comienza liberado para que lo pueda tomar quien quiera
}
