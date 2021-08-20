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
#include "../include/common.h"

static void Task_IRQ_UART_SIM(void *pvParameters);

//extern const char image[];
extern QueueHandle_t uart1_queue, Cola_Resp_Tarj,queue_fot_wr,queue_rec_sd;
extern SemaphoreHandle_t Mutex_UART_LTE, Mutex_Trama_GPS, sem_sim_rdy, sem_sim_ok;
extern uint8_t MAC[6];
extern uint32_t RFID;
//------------------------------------------------------------------------------------
// Open_Com_TCP
//      Abre la comunicacion TCP
//------------------------------------------------------------------------------------
void Open_Com_TCP(void)
{
    uart_write_bytes(UART_SIM, "AT+NETOPEN\r", sizeof("AT+NETOPEN\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, IP_PORT, strlen(IP_PORT));
    xSemaphoreTake(sem_sim_ok,60000/portTICK_PERIOD_MS);     //ACA tengo que recibir CONNECT 921600
}


//------------------------------------------------------------------------------------
// Close_Com_TCP
//      Cierra la comunicacion TCP
//------------------------------------------------------------------------------------
void Close_Com_TCP(void)
{
    uart_write_bytes(UART_SIM, "AT+CIPCLOSE=0\r", sizeof("AT+CIPCLOSE=0\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    vTaskDelay(300/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, "AT+NETCLOSE\r", sizeof("AT+NETCLOSE\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    vTaskDelay(300/portTICK_PERIOD_MS);
}


//------------------------------------------------------------------------------------
// Set_NonTransparent
//      Configura el modo no transparente
//------------------------------------------------------------------------------------
void Set_NonTransparent(void)
{
    uart_write_bytes(UART_SIM, "AT+CIPMODE=0\r", sizeof("AT+CIPMODE=0\r")-1);    //inicio modo no transparente
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, "AT+NETOPEN\r", sizeof("AT+NETOPEN\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, IP_PORT, strlen(IP_PORT));
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
}
//------------------------------------------------------------------------------------
// Send_NonTransparent
//      Envia en modo no transparente
//------------------------------------------------------------------------------------
void Send_NonTransparent(const char* data)
{
    const char finTrama = 0x1A;         //Fin de trama a enviar -> CTRL+Z

    //Enviar string que termina con CTRL Z (enter)
    uart_write_bytes(UART_SIM, "AT+CIPSEND=0,\r", sizeof("AT+CIPSEND=0,\r")-1);
    ESP_LOGI("SIM", "\n\n--%s--\n\n",data); 
    uart_write_bytes(UART_SIM, data, strlen(data)-1);
    uart_write_bytes(UART_SIM, &finTrama, sizeof(finTrama));

    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
}

// End_NonTransparent
//      Cierra la comunicación no transparente
//------------------------------------------------------------------------------------
void End_NonTransparent (void)
{
    Close_Com_TCP();
}


//------------------------------------------------------------------------------------
// Set_Transparent
//       Configura el modo transparente
//------------------------------------------------------------------------------------
void Set_Transparent(void)
{
    xSemaphoreTake(sem_sim_ok,200/portTICK_PERIOD_MS);   //por si quedo algun semaforo liberado
    uart_write_bytes(UART_SIM, "AT+CIPMODE=1\r", sizeof("AT+CIPMODE=1\r")-1);    //inicio modo transparente
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    Open_Com_TCP();         //Abro la comunicacion
}


//------------------------------------------------------------------------------------
// Send_Transparent
//       Envia en modo transparente
//------------------------------------------------------------------------------------
void Send_Transparent(const char* data,uint32_t size)
{   
    const uint16_t bytes_cons=256;
    uint32_t i,j;
    for(i=0,j=0;size-i > bytes_cons;i=i+bytes_cons,j=j+bytes_cons)  //Envio cada una cierta cantidad de bytes por limitación de uart write
    {
        uart_write_bytes(UART_SIM, &data[i],bytes_cons);
        if(j>10000)
        {
            vTaskDelay(250/portTICK_PERIOD_MS);
            j=0;
        }            
    }
    uart_write_bytes(UART_SIM, &data[i],size-i);  //envio los bytes restantes
}


//------------------------------------------------------------------------------------
// End_Transparent
//       Termina el modo transparente
//------------------------------------------------------------------------------------

void End_Transparent(void)
{
    uart_wait_tx_done(UART_SIM, portMAX_DELAY);
    
    vTaskDelay(1500/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM,"+",sizeof("+")-1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM,"+",sizeof("+")-1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM,"+",sizeof("+")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    Close_Com_TCP();        //Cierro la comunicacion
    Close_Com_TCP();        //Cierro la comunicacion
}

void Change_NonTransparent(void)
{
    uart_wait_tx_done(UART_SIM, portMAX_DELAY);
    vTaskDelay(1500/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM,"+",sizeof("+")-1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM,"+",sizeof("+")-1);
    vTaskDelay(100/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM,"+",sizeof("+")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
}

void Change_Transparent(void)
{
    uart_write_bytes(UART_SIM,"ATO\r",sizeof("ATO\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);  // Acá tengo que recibir CONNECT 921600
}

//------------------------------------------------------------------------------------
// Get_GPS
//       Obtiene la trama GPS
//  +CGPSINFO: 3433.035574,S,05829.585934,W,310520,210511.0,6.6,0.0,0.0        
//  Latitud, N/S, Longitud, W/E, Fecha, Hora.central, Altitud, Velocidad, Curso 
//------------------------------------------------------------------------------------
void Get_GPS (void)
{
    //uart_write_bytes(UART_SIM, "AT+CGPSINFO\r", sizeof("AT+CGPSINFO\r")-1);  //Obtener info del GPS  
    //uart_write_bytes(UART_SIM, "AT+CNETSTART\r", sizeof("AT+CNETSTART\r")-1);  //Obtener info del GPS x triangulacion
    //vTaskDelay(500/portTICK_PERIOD_MS);
    
    uart_write_bytes(UART_SIM, "AT+CLBS=4\r", sizeof("AT+CLBS=4\r")-1);  //Obtener info del GPS x triangulacion
}


//------------------------------------------------------------------------------------
// Analizar_Trama
//      Analiza las tramas que llegan por UART
//------------------------------------------------------------------------------------
void Analizar_Trama(char *dtmp)
{
    const char gps [] = "+CLBS: ";
    const char rdy[] = "PB DONE";
    const char closed[]= "CLOSED";
    const char ok[]="OK";
    const char connect[]="CONNECT";

    if(strstr(dtmp,rdy) != NULL)
    {
        ESP_LOGI("SIM","Llego el ready");        
        xSemaphoreGive(sem_sim_rdy);
    }

    else if(strstr(dtmp,closed) != NULL)    
    {
        ESP_LOGI("SIM","Se desconecto del server -- Reconectando");        
        //xSemaphoreTake(Mutex_UART_LTE,portMAX_DELAY);
        ESP_LOGI("SIM","Salio de modo transparente");        
        fatal_error();
    }

    else if(strstr(dtmp,ok) != NULL)        
    {
        ESP_LOGI("SIM","Se recibio el OK");        
        xSemaphoreGive(sem_sim_ok);
    }    

    else if(strstr(dtmp,TR_IM) != NULL)        
    {
        ESP_LOGI("SIM","Se recibio el pedido de foto");        
        uint8_t data=3;	// es una foto de deslogueo
        xQueueSendToBack(queue_fot_wr,&data,portMAX_DELAY);    
    } 

    else if(strstr(dtmp,TR_RFID) != NULL)        
    {
        ESP_LOGI("SIM","Se recibio la contestación del RFID");        
        if(strstr(dtmp,"registrado") != NULL)   //Tarjeta aceptada
        {
            uint8_t data=1;
            xQueueOverwrite(Cola_Resp_Tarj,&data);
        }     
        else
        {
            uint8_t data=0;
            xQueueOverwrite(Cola_Resp_Tarj,&data);
        }
    } 

    else if(strstr(dtmp,TR_VID) != NULL)        
    {
        ESP_LOGI("SIM","Se recibio el pedido de video");        
        uint8_t data=2;  //significa pedido
        xQueueSendToBack(queue_rec_sd,&data,portMAX_DELAY);    
    } 

    if(strstr(dtmp,connect) != NULL)
    {
        ESP_LOGI("SIM","Se conecto al server");
        xSemaphoreGive(sem_sim_ok);
        //xSemaphoreGive(Mutex_UART_LTE);
    }
}


//------------------------------------------------------------------------------------
// Init_LTE
//      
//------------------------------------------------------------------------------------
void Init_LTE(uint8_t estado)
{    
    uart_write_bytes(UART_SIM,"ATE1\r",sizeof("ATE1\r")-1);  //Habilitar: ATE1 | Deshabilitar: ATE0
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);

    uart_write_bytes(UART_SIM, "AT+CSQ\r", sizeof("AT+CSQ\r")-1);    //AT+CSQ -> Check signal quality 
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, "AT+CREG?\r", sizeof("AT+CREG?\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, "AT+CPSI?\r", sizeof("AT+CPSI?\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, "AT+CGDCONT=1,\"IP\",\"CMNET\"\r", sizeof("AT+CGDCONT=1,\"IP\",\"CMNET\"\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
/*
//inicio modo no transparente

    uart_write_bytes(UART_SIM, "AT+CIPMODE=0\r", sizeof("AT+CIPMODE=0\r")-1);    
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);

    uart_write_bytes(UART_SIM, "AT+CSOCKSETPN=1\r", sizeof("AT+CSOCKSETPN=1\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, "AT+NETOPEN\r", sizeof("AT+NETOPEN\r")-1);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    uart_write_bytes(UART_SIM, IP_PORT, strlen(IP_PORT));
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);

    vTaskDelay(2000/portTICK_PERIOD_MS);
        
    Close_Com_TCP();
*/
    Turn_off_gpio_SIM(LEDV);
    Set_Transparent();
    Estado_Dispositivo(estado);
}


//------------------------------------------------------------------------------------
// Task_IRQ_UART_1_Init
//      Manejo de IRQ de la UART_1
//------------------------------------------------------------------------------------
static void Task_IRQ_UART_SIM(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = malloc(SIM_BUF_SIZE);
    int len = 0;

    while(1) 
    {
        //Espera por una interrupcion
        if(xQueueReceive(uart1_queue, (void * )&event, (portTickType)portMAX_DELAY)) 
        {            
            bzero(dtmp, SIM_BUF_SIZE);

            switch(event.type) 
            {
                case UART_DATA:
                    len = uart_read_bytes(UART_SIM, (uint8_t*) dtmp, event.size, portMAX_DELAY);
                    if(len > 0)
                    {
                        //dtmp[len] = '\0';
                        ESP_LOGI("SIM", "%s",dtmp);
                    }
                    Analizar_Trama((char *)dtmp);        
                break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI("UART1", "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_SIM);
                    xQueueReset(uart1_queue);
                break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI("UART1", "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_SIM);
                    xQueueReset(uart1_queue);
                break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI("UART1", "uart rx break");
                break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI("UART1", "uart parity error");
                break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI("UART1", "uart frame error");
                break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_SIM, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_SIM);
                    ESP_LOGI("UART1", "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) 
                    {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_SIM);
                    } 
                    else 
                    {
                        uart_read_bytes(UART_SIM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_SIM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI("UART1", "read data: %s", dtmp);
                        ESP_LOGI("UART1", "read pat : %s", pat);
                    }
                break;
                //Others
                default:
                    ESP_LOGI("UART1", "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;

    vTaskDelete(NULL);
}

//------------------------------------------------------------------------------------
// Init_UART_1
//      Inicializacion de la UART_1
//------------------------------------------------------------------------------------
void Init_SIM(void)
{
    /* Configure parameters of an UART driver, communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = UART_SIM_VEL,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_SIM, &uart_config);

    //Set UART log level
    esp_log_level_set("UART1", ESP_LOG_INFO);
    
    //Set UART pins (using UART0 default pins ie no changes)
    uart_set_pin(UART_SIM, TX1, RX1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Install UART driver, and get the queue
    uart_driver_install(UART_SIM, SIM_BUF_SIZE/2, SIM_BUF_SIZE, 20, &uart1_queue, 0);

    //Tarea para manejar la UART desde interrupcion
    xTaskCreate(Task_IRQ_UART_SIM, "uart_event_task_init", 2048, NULL, tskIDLE_PRIORITY + 5UL, NULL);

    #ifdef  GPS_CONECTADO
        reset_SIM();
    #endif

}


void change_sim_vel(uint32_t velocidad)
{
    char temp[30]={0};

    sprintf(temp,"AT+IPR=%u\r",velocidad);

    uart_write_bytes(UART_SIM,temp,strlen(temp));        //Configuro el baudrate del SIM
    
    vTaskDelay(1000/portTICK_PERIOD_MS);    //Delay

    uart_write_bytes(UART_SIM,"AT+IPR?\r",sizeof("AT+IPR?\r")-1);    //Pregunto para ver si el baudrate se configuro
    
    vTaskDelay(1000/portTICK_PERIOD_MS);    //Delay
}


//------------------------------------------------------------------------------------
// Turn_on_gpio_SIM
//      Activa el GPIO del SIM 
//------------------------------------------------------------------------------------
void Turn_on_gpio_SIM (uint8_t pin)
{
    char temp[25]={0};
    sprintf(temp,"AT+CGSETV=%d,1\r",pin);   // PIN, VALOR value=0 -> LOW --- Value=1 -> HIGH 
    uart_write_bytes(UART_SIM, temp, strlen(temp));
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
}

//------------------------------------------------------------------------------------
// Turn_off_gpio_SIM
//      Desactiva el GPIO del SIM
//------------------------------------------------------------------------------------
void Turn_off_gpio_SIM (uint8_t pin)
{
    char temp[25]={0};
    sprintf(temp,"AT+CGSETV=%d,0\r",pin);
    uart_write_bytes(UART_SIM, temp, strlen(temp));
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
}

//------------------------------------------------------------------------------------
// Input_Output_Init
//      Inicializa entradas y salidas
//------------------------------------------------------------------------------------
void Init_Input_Output_sim(void)
{
    char temp[50]={0};

    sprintf(temp,"AT+CGDRT=%d,1\r",LEDV);       //0 es IN - 1 es OUT
    uart_write_bytes(UART_SIM, temp, strlen(temp)); 
    bzero(temp,50);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    sprintf(temp,"AT+CGDRT=%d,1\r",LEDR);        //0 es IN - 1 es OUT
    uart_write_bytes(UART_SIM, temp, strlen(temp)); 
    bzero(temp,50);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    sprintf(temp,"AT+CGDRT=%d,1\r",LEDFLASH);    //0 es IN - 1 es OUT
    uart_write_bytes(UART_SIM, temp, strlen(temp)); 
    bzero(temp,50);
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);
    sprintf(temp,"AT+CGDRT=%d,1\r",BUZZER);     //0 es IN - 1 es OUT
    uart_write_bytes(UART_SIM , temp, strlen(temp)); 
    xSemaphoreTake(sem_sim_ok,5000/portTICK_PERIOD_MS);

    Turn_on_gpio_SIM(LEDR);
    Turn_on_gpio_SIM(LEDV);
    ESP_LOGD("SIM","Iniciados los pines de la sim");
}

//------------------------------------------------------------------------------------
// ArmadoTrInicio
//      Armo la trama para enviar la inicialización de la comunicación
//------------------------------------------------------------------------------------
void ArmadoTrInicio(char* devolucion,char* TipoTrama)
{
    //char temp[70];
    sprintf(devolucion,"%s,%x%x%x%x%x%x,%u,%s",TR_IN,(uint8_t)MAC[0],(uint8_t)MAC[1],(uint8_t)MAC[2],(uint8_t)MAC[3],(uint8_t)MAC[4],(uint8_t)MAC[5],RFID,TipoTrama);
    //strcpy(devolucion,temp);
}

//------------------------------------------------------------------------------------
// Task_Send_Basura
//      Envió basura periodicamente para vaciar el buffer del SIM
//------------------------------------------------------------------------------------
void Task_Send_Basura(void *pvParameter)
{
    const char temp[]="Tracking system: Connection established";

    while(1)
    {
        vTaskDelay(500/portTICK_PERIOD_MS);
        xSemaphoreTake(Mutex_UART_LTE,portMAX_DELAY);            
        Send_Transparent(temp,strlen(temp));
        xSemaphoreGive(Mutex_UART_LTE);
    }
}

//------------------------------------------------------------------------------------
// Estado_Dispositivo
//      Envió al inicializar el dispositivo si hay error con algún módulo
//------------------------------------------------------------------------------------
void Estado_Dispositivo (uint8_t estado)
{
    char envio[100],init[35],disp[30];

    ArmadoTrInicio(init,AL_DISP);

    if(estado == 0)
        strcpy(disp,"OK");
    else if(estado == 1)
        strcpy(disp,"MICRO");
    else if(estado == 2)
        strcpy(disp,"CAMARA");
    else if(estado == 3)
        strcpy(disp,"SD");
    else if(estado == 4)
        strcpy(disp,"RFID");
    else if(estado == 5)
        strcpy(disp,"ACELEROMETRO");

    sprintf(envio,"%s,%s,%s",init,disp,TR_FIN);
    uart_write_bytes(UART_SIM, envio, strlen(envio));

    if(estado != 0)
        fatal_error();
}

//------------------------------------------------------------------------------------
// Reseteo SIM
//      Reseteo el SIM con el PWRKEY
//------------------------------------------------------------------------------------
void reset_SIM()
{

    //Inicialización pin de salida

    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1UL << PWR_SIM;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    // Reseteo con tiempos del manual
    gpio_set_level(PWR_SIM,0);
    vTaskDelay(500/portTICK_PERIOD_MS);
    gpio_set_level(PWR_SIM,1);
}