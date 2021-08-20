#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_camera.h"

#include "driver/uart.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "ff.h"

#include "../include/SD/sd.h"
#include "../include/camera/camera.h"
#include "../include/SIM/sim.h"
#include "../include/common.h"


/**********************/
// Variables externas
/**********************/

static const char *SD_TAG = "SD";

extern xSemaphoreHandle mutex_sd, Mutex_UART_LTE;
extern xSemaphoreHandle sem_fin_foto,sem_fot_dis;
extern QueueHandle_t queue_fot_send,queue_fot_wr,queue_rec_sd;

//static long long tick_if[2];
extern camera_fb_t* imagen[TAM_VEC_FOTOS];
extern esp_timer_handle_t periodic_timer;

static uint16_t latest_video = 0; // Variable global para saber cual fue el ultimo video cerrado

/**********************/
// Funciones
/**********************/

/*********************************************************************************************************/
//Inicializa la SD

esp_err_t sd_init_sdmode()
{
    ESP_LOGI(SD_TAG, "Initializing SD card");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();   //El clock ya este definido en pin 14
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;    // To use 1-line SD mode
    gpio_set_pull_mode(SD_CMD, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(SD_DATA0, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    // If format_if_mount_failed is set to true, SD card will be partitioned and formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 32 * 1024
    };

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Please check its source code and implement error recovery when developing production applications.
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) 
    {
        if (ret == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
                return ESP_FAIL;
        } else {
            ESP_LOGE(SD_TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
                return ESP_FAIL;
        }
    }
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(SD_TAG,"Termino el init de la SD");   
    return ESP_OK;
}

void create_sd_tasks(void)
{
    xTaskCreatePinnedToCore(&sd_write, "sd_write", 8 * 1024,NULL,tskIDLE_PRIORITY+3,NULL,1);  //ASIGNADA A CORE 1!!! 
    xTaskCreatePinnedToCore(&sd_read, "sd_read", 8 * 1024 , NULL,tskIDLE_PRIORITY+4, NULL, 1); // Tarea read con prioridad mayor a write
    ESP_LOGI(SD_TAG,"Se crearon las tareas de la SD card"); 
}

/*********************************************************************************************************/
// Tarea encargada en leer el buffer de fotos y guardarlos en la SD


void sd_write(void *pvParameter)
{
    uint8_t fot_save=0; 
    uint16_t num_video=0;
    while(1)
    {
        char * name_video;
        FILE * video_file;
            
        asprintf(&name_video,"/sdcard/%.4d.avi",num_video);

        ESP_LOGI(SD_TAG,"El tipo de frame es: %s",name_video);
            
        xSemaphoreTake(mutex_sd,portMAX_DELAY);

        remove(name_video);
        video_file = fopen(name_video, "w");
        
        if(video_file == NULL)
        {
            ESP_LOGE (SD_TAG,"No se pudo crear el archivo %s. Se reiniciara en 5 seg",name_video);
            fatal_error();
        }
        free(name_video);

        xSemaphoreGive(mutex_sd);

        for(uint16_t i=0; i<NRO_FOTOS/(FOTOS_CONS);i++)
        {
            xSemaphoreTake(sem_fot_dis, portMAX_DELAY);

            for(int j=0; j<FOTOS_CONS;j++,fot_save++)
            {
                if(fot_save >= TAM_VEC_FOTOS)
                    fot_save =0;
                uint32_t lenght_fi;
                lenght_fi = imagen[fot_save]->len;
                lenght_fi += ((0x4 - (0x3 & lenght_fi)) & 0x3) + 4; //longitud de la imagen + relleno +4 bytes de separador final
                
                xSemaphoreTake(mutex_sd,portMAX_DELAY);

                fwrite(&(imagen[fot_save]->len),sizeof(uint32_t),1,video_file);
                if((i*FOTOS_CONS+j) == (NRO_FOTOS-1))
                {
                    fwrite(imagen[fot_save]->buf,sizeof(uint8_t),lenght_fi-4,video_file);//el separador lo escribo aparte
                    fwrite("\255\255\255\255",sizeof(char),4,video_file);
                }
                else    //escribo la trama + relleno + separador con lo que haya en el buffer
                    fwrite(imagen[fot_save]->buf,sizeof(uint8_t),lenght_fi,video_file);

                
                //Codigo para guardar foto unica bajo cualquier condicion (bajo pedido, 60 seg o rfid)
                //foto_unica.jpeg
                //Una vez finalizada envia señal a sd read
                uint8_t dato=0;
                if(xQueueGenericReceive(queue_fot_wr,&dato,0,false) == pdTRUE)
                {
                    ESP_LOGI("SD","Entramos al if de foto unica");

                    FILE * ffot_unica=NULL;
                    ffot_unica=fopen("/sdcard/f_un.dat","w");
                    if(ffot_unica==NULL)
                        ESP_LOGE("SD","ERROR EN OPEN ffot_unica");
                    fwrite(&(imagen[fot_save]->len),sizeof(uint32_t),1,ffot_unica);
                    fwrite(imagen[fot_save]->buf,sizeof(uint8_t),lenght_fi-4,ffot_unica);
                    fwrite("\255\255\255\255",sizeof(char),4,ffot_unica);
                    fclose(ffot_unica);
                    xQueueSendToBack(queue_fot_send,&dato,portMAX_DELAY);
                    ESP_LOGI("SD","Se libero semaphore sem_foto_unica_rd");
                }
                
                esp_camera_fb_return(imagen[fot_save]);
                xSemaphoreGive(mutex_sd);
            }
        }

        xSemaphoreTake(mutex_sd,portMAX_DELAY);
        fclose(video_file);
        xSemaphoreGive(mutex_sd);

        latest_video=num_video;
        num_video ++;  
        
        if(num_video >= VIDEOS_RESET)
            num_video = 0;
    }

    esp_vfs_fat_sdmmc_unmount();
    ESP_LOGE(SD_TAG, "Card unmounted");
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_LOGE("TIMER_CAM","Timer y camaras detenidos");
    fatal_error();
    vTaskDelete(NULL); 
}

/*********************************************************************************************************/
// Tarea encargada en leer la SD y enviar sea una foto única o un video

void sd_read(void *pvParameter)
{
    //static long long tick_if_read[2];
    const uint16_t tam_lect_sd = (SIM_BUF_SIZE/2);
    uint8_t dato=0;

    while(1)
    {
        if(xQueueReceive(queue_rec_sd,&dato,0)==pdTRUE)    //si no esta disponible retorna instantaneamente
        {
            char * name_video;
            FILE * video_file;
            uint16_t nombre_us;

            xSemaphoreTake(Mutex_UART_LTE, portMAX_DELAY);  //Tomo el semaforo
            ESP_LOGI("SD", "Empezando a leer y enviar por uart video");
            
            char temp[100],inicio[150];

            ArmadoTrInicio(temp,TR_VID);
            if(dato == 1)
                sprintf(inicio,"%s,%s,",temp,"emergencia");
            else if(dato == 2)
                sprintf(inicio,"%s,%s,",temp,"solicitud");

            Send_Transparent(inicio,strlen(inicio));

            uint16_t latest_video_temp = latest_video;

            char *video_string = malloc(tam_lect_sd * sizeof(uint8_t));   //chequear que sea menor al tamaño del buffer creado

            for(int8_t j=(TIEMPO_ENVIAR/(NRO_FOTOS/FPS_CAM))-1;j>=0;j--)   //para enviar 30 seg por uart
            {
                int resta = (int)(latest_video_temp - j);
                if(resta < 0)
                    nombre_us = (uint16_t)(VIDEOS_RESET + resta + 1); 
                else
                    nombre_us = latest_video_temp - j;

                asprintf(&name_video,"/sdcard/%.4d.avi", nombre_us);
                xSemaphoreTake(mutex_sd,portMAX_DELAY);
                video_file = fopen(name_video, "r");
                fseek(video_file, 0, SEEK_END);
                uint32_t fsize = ftell(video_file);
                fseek(video_file, 0, SEEK_SET);  /* same as rewind(f); */
                xSemaphoreGive(mutex_sd);

                for(int i=0; i<(fsize/(tam_lect_sd)); i++)
                {
                    xSemaphoreTake(mutex_sd,portMAX_DELAY);
                    if(tam_lect_sd != fread(video_string, sizeof(uint8_t), tam_lect_sd, video_file))
                        ESP_LOGW("SD","Lei mal en %ld", ftell(video_file));
                    xSemaphoreGive(mutex_sd);
                    Send_Transparent(video_string,tam_lect_sd);
                    vTaskDelay(70/portTICK_PERIOD_MS);                    
                }
                uint32_t f_act = ftell(video_file);
                if(f_act != fsize)
                {
                    xSemaphoreTake(mutex_sd,portMAX_DELAY);
                    if((fsize-f_act)!=fread(video_string, sizeof(uint8_t), fsize-f_act, video_file)) 
                        ESP_LOGW("SD","Lei mal en %d",f_act);
                    fclose(video_file);
                    xSemaphoreGive(mutex_sd);
                    Send_Transparent(video_string,fsize-f_act);                                        
                }
            }
                ESP_LOGI("SIM","Enviando trama de finalización");
                char fin[20];
                sprintf(fin,",%s",TR_FIN);
                Send_Transparent(fin,strlen(fin));
                xSemaphoreGive(Mutex_UART_LTE);     //Doy el semaforo
                free(video_string);
        }

        //Codigo de envio de imagen unica sea bajo pedido, 60 seg o rfid (semaforo dado por sdwrite)
        if(xQueueGenericReceive(queue_fot_send,&dato,100/portTICK_RATE_MS,false)==pdTRUE) //si no esta disponible en 100ms retorna
        {

            FILE *photo_file=NULL;
            xSemaphoreTake(mutex_sd,portMAX_DELAY);
            photo_file = fopen("/sdcard/f_un.dat", "r");
            if(photo_file == NULL)
            {
                ESP_LOGW("SD","Error en open f_un.dat");
                break;
            }

            xSemaphoreGive(mutex_sd);

            fseek(photo_file, 0, SEEK_END);
            uint32_t fsize = ftell(photo_file);
            fseek(photo_file, 0, SEEK_SET);

            char *photo_string = malloc(tam_lect_sd * sizeof(uint8_t));   //chequear que sea menor al tamaño del buffer creado

            xSemaphoreTake(Mutex_UART_LTE, portMAX_DELAY);  //Tomo el semaforo
            //Envio por UART del paquete
            ESP_LOGI("SD", "Empezando a leer y enviar por uart imagen simple");
            char temp[100],inicio[150];

            ArmadoTrInicio(temp,TR_IM);
            if(dato == 1)
                sprintf(inicio,"%s,%s,",temp,"captura");
            else if(dato == 2)
                sprintf(inicio,"%s,%s,",temp,"logueo");
            else if(dato == 3)
                sprintf(inicio,"%s,%s,",temp,"solicitud");
            else if(dato == 4)
                sprintf(inicio,"%s,%s,",temp,"deslogueo");

            Send_Transparent(inicio,strlen(inicio));

            for(int i=0; i<(fsize/(tam_lect_sd)); i++)
            {
                xSemaphoreTake(mutex_sd,portMAX_DELAY);
                if(tam_lect_sd != fread(photo_string, sizeof(uint8_t), tam_lect_sd, photo_file))
                    ESP_LOGW("SD","Lei mal en %ld", ftell(photo_file));
                xSemaphoreGive(mutex_sd);
                Send_Transparent(photo_string,tam_lect_sd);
            }
            uint32_t f_act = ftell(photo_file);
            if(f_act != fsize)
            {
                xSemaphoreTake(mutex_sd,portMAX_DELAY);
                if((fsize-f_act)!=fread(photo_string, sizeof(uint8_t), fsize-f_act, photo_file)) 
                    ESP_LOGW("SD","Lei mal en %d",f_act);
                fclose(photo_file);
                xSemaphoreGive(mutex_sd);
                Send_Transparent(photo_string,fsize-f_act);                    
            }

            ESP_LOGI("SD", "Enviada trama fin por uart imagen simple");
            char fin[20];
            sprintf(fin,",%s",TR_FIN);
            Send_Transparent(fin,strlen(fin));

            xSemaphoreGive(Mutex_UART_LTE);     //Doy el semaforo
            free(photo_string);
        }
        
    }

    esp_vfs_fat_sdmmc_unmount();
    ESP_LOGE(SD_TAG, "Card unmounted");
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_LOGE("TIMER_CAM","Timer y camaras detenidos");
    fatal_error();
    vTaskDelete(NULL);
}