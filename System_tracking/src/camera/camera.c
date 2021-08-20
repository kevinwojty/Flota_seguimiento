#include "../include/camera/camera.h"
#include "esp_camera.h"
#include "../include/common.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "string.h"

static const char* CAMERA = "camara";

extern xSemaphoreHandle sem_fot_dis,sem_take_pic,sem_rec_sd;
extern QueueHandle_t queue_fot_wr;
extern camera_fb_t* imagen[TAM_VEC_FOTOS];
esp_timer_handle_t periodic_timer;

static camera_config_t camera_config =
{
  .pin_pwdn  = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_VGA,//QQVGA-QXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = JPEG_QUALITY, //0-63 lower number means higher quality
  .fb_count = 1 + FB_MAX //if more than one, i2s runs in continuous mode. Use only with JPEG
};

/*****************************************************************************************************/
//Inicialización de la cámara y sus parametros
esp_err_t camera_init_main ()
{
  //power up the camera if PWDN pin is defined
  if(PWDN_GPIO_NUM != -1){
      gpio_set_direction(PWDN_GPIO_NUM, GPIO_MODE_OUTPUT);
      gpio_set_level(PWDN_GPIO_NUM, 0);
  }

  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) 
  {
    ESP_LOGE(CAMERA,"Camera Init Failed");
    return ESP_FAIL;
  }
  xTaskCreatePinnedToCore(&camera_per_im, "Cam_per_im", 10 * 1024,NULL,tskIDLE_PRIORITY+5,NULL,0);  //ASIGNADA A CORE 0
  ESP_LOGI(CAMERA,"Init cam terminado");
  return ESP_OK;
}

/*****************************************************************************************************/
//Saca fotos periodicamente cada X tiempo y le pasa por una variable el puntero a la SD
void camera_per_im (void *pvParameter)
{
	camera_fb_t * fb = NULL;

   for(int i=0;i<50;i++)   //desecho las primeras 50 fotos para permitir que se acostumbre a la luz la camara
        {
            fb = esp_camera_fb_get();
            while (fb == NULL) 
            {
                ESP_LOGW("CAMERA","Camera capture failed");
                fb = esp_camera_fb_get();
            }
            esp_camera_fb_return(fb);
        }
  static uint8_t cant_fotos=0;
  static uint16_t total_fotos=0;
	while(1)
	{
    
    for(int i=0;i<FOTOS_CONS;i++,total_fotos++,cant_fotos++)
    {
      if(cant_fotos >= TAM_VEC_FOTOS)
        cant_fotos = 0; //reseteamos el contador que barre el vector
      xSemaphoreTake(sem_take_pic,portMAX_DELAY);
      //ESP_LOGI(CAMERA,"Tomado el sem_take_pic");
      fb=esp_camera_fb_get();
      size_t tam = fb->len;
      tam += ((0x4 - (0x3 & tam)) & 0x3); //relleno + tamaño de la foto
      imagen[cant_fotos]= fb;
    }
    xSemaphoreGive(sem_fot_dis);
	}
  
  ESP_LOGW(CAMERA,"Saliendo tarea camara");
  vTaskDelete(NULL);  //llegue al fin de la inicialización
}

/*****************************************************************************************************/
//Inicialización del timer periódico
void mytimer_init(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
                .callback = &periodic_timer_callback,
                /* name is optional, but may help identify the timer when debugging */
                .name = "Per_Cam"
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */

    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 1000000/FPS_CAM));// Se repite cada 1 Seg/FPS

    ESP_LOGI("TIMER", "Timer inicializado");
}


/*****************************************************************************************************/
//Timer periodico para indicar a la camara cuando capturar la siguiente foto
static void periodic_timer_callback(void* arg)
{
    static uint16_t contador;

    xSemaphoreGive(sem_take_pic);
    contador++;
    if(contador > FPS_CAM * 60) // Se dispara cada 1 minuto
    {
      contador = 0;
      uint8_t data=1; // es captura
      xQueueSendToBack(queue_fot_wr,&data,portMAX_DELAY);
    }
}