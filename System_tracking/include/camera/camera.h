#include "esp_err.h"
#include "esp_log.h"

/************* Defines *******************/
#define NRO_VIDEOS      999999
#define FOTOS_CONS      2    // cantidad de fotos consecutivas que se van a guardar
#define TAM_VEC_FOTOS   3
#define FB_MAX          TAM_VEC_FOTOS-1  //Para cambiar la máxima cantidad de fotos que se pueden almacenar en el buffer de la cámara
#define FPS_CAM         9
#define TIEMPO_ENVIAR   6//12
#define TIEMPOxVIDEO    6   // tiempo en segundos que se almacen en cada video 
#define NRO_FOTOS       TIEMPOxVIDEO * FPS_CAM  // tiene que ser multiplo de fotos_cons, tam_vec_fotos y mayor a FPS_CAM 


#define JPEG_QUALITY    12
#define TAM_SD_GB       16
#define TAM_1SEG_KB     225
#define VIDEOS_RESET    50 //TAM_SD_GB*1048576/(TAM_1SEG_KB*(NRO_FOTOS/FPS_CAM))    //chequear que no supere 9999 (limitación de código al crear nombre de archivos)


/************* Cam Setup *******************/
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       36 //Invertido en la placa respecto a la original 39
#define Y6_GPIO_NUM       39 //Invertido en la placa respecto a la original 36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

/********************************************************************************************************
 * Declaracion de funciones
 */
esp_err_t camera_init_main ();
void camera_per_im (void *pvParameter);
void mytimer_init(void);
static void periodic_timer_callback(void* arg);
