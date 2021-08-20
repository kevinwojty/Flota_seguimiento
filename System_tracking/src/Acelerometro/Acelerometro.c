#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "driver/spi_master.h"

//#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h" 
#include "esp_intr_alloc.h" 

#include "../include/Acel/bma400.h"
#include "../include/Acel/bma400_defs.h"
#include "../include/Acel/Acelerometro.h"
#include "../include/common.h"
#include "../include/SIM/sim.h"

static struct bma400_dev *_bma400_dev;
static spi_device_handle_t accel_spi;

extern xSemaphoreHandle Mutex_UART_LTE;
extern QueueHandle_t queue_rec_sd;

//El BMA envia un dumy bite antes de responder, a tener en cuenta!!
BMA400_INTF_RET_TYPE accel_read (uint8_t reg_addr, uint8_t *reg_data, uint32_t length,void *intf_ptr)
{

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.flags = SPI_TRANS_USE_TXDATA; // | SPI_TRANS_USE_RXDATA;
    t.length = 8;//Length es la cantidad de BITS que quiero escribir
    t.tx_data[0] = reg_addr;
    t.rxlength = length*8;  //el LENGTH parámetro viene dado en BYTES!
    t.rx_buffer = reg_data;

    esp_err_t ret = spi_device_transmit(intf_ptr, &t);
    assert(ret == ESP_OK);

    //for(uint8_t i=1;i<length;i++)   
        //ESP_LOGI("SPI","Recibido %x ",reg_data[i]);

    return BMA400_INTF_RET_SUCCESS;
}


BMA400_INTF_RET_TYPE accel_write (uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,void *intf_ptr)
{

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    t.length = length*8+8;
    //t.tx_buffer = reg_data;
    t.flags = SPI_TRANS_USE_TXDATA; // | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = reg_addr;
    t.tx_data[1] = *reg_data;
    esp_err_t ret = spi_device_transmit(intf_ptr, &t);

    if(ret != ESP_OK)
        return BMA400_E_COM_FAIL;

    return BMA400_INTF_RET_SUCCESS;
}

void delay_us (uint32_t period, void *intf_ptr)
{
    vTaskDelay(period/(portTICK_PERIOD_MS*1000));
}


static void bma400_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMA400_OK:

            /* Do nothing */
            break;
        case BMA400_E_NULL_PTR:
            ESP_LOGI("ACCEL","Error [%d] : Null pointer\r\n", rslt);
            break;
        case BMA400_E_COM_FAIL:
            ESP_LOGI("ACCEL","Error [%d] : Communication failure\r\n", rslt);
            break;
        case BMA400_E_INVALID_CONFIG:
            ESP_LOGI("ACCEL","Error [%d] : Invalid configuration\r\n", rslt);
            break;
        case BMA400_E_DEV_NOT_FOUND:
            ESP_LOGI("ACCEL","Error [%d] : Device not found\r\n", rslt);
            break;
        default:
            ESP_LOGI("ACCEL","Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

static float lsb_to_ms2(int16_t accel_data, uint8_t g_range, uint8_t bit_width)
{
    float accel_ms2;
    int16_t half_scale;

    half_scale = 1 << (bit_width - 1);
    accel_ms2 = (GRAVITY_EARTH * accel_data * g_range) / half_scale;

    return accel_ms2;
}


static void get_accel ()
{
    uint8_t n_samples=10;
    uint8_t rslt=BMA400_OK;
    uint16_t int_status = 0;
    struct bma400_sensor_data data;
    float t, x, y, z;
    struct bma400_sensor_conf conf;
    struct bma400_int_enable int_en;

    conf.type = BMA400_ACCEL;

    // Modify the desired configurations as per macros
    // available in bma400_defs.h file /
    conf.param.accel.odr = BMA400_ODR_100HZ;
    conf.param.accel.range = BMA400_RANGE_2G;
    conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    // Set the desired configurations to the sensor /
    rslt = bma400_set_sensor_conf(&conf, 1, _bma400_dev);
    bma400_check_rslt("bma400_set_sensor_conf", rslt);
    ESP_LOGI("ACCEL","SET sensor conf paso");

    // Get the accelerometer configurations which are set in the sensor /
    rslt = bma400_get_sensor_conf(&conf, 1, _bma400_dev);
    bma400_check_rslt("bma400_get_sensor_conf", rslt);

    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, _bma400_dev);
    bma400_check_rslt("bma400_set_power_mode", rslt);
    ESP_LOGI("ACCEL","SET power mode paso");

    int_en.type = BMA400_DRDY_INT_EN;
    int_en.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&int_en, 1, _bma400_dev);
    bma400_check_rslt("bma400_enable_interrupt", rslt);

    ESP_LOGI("ACCEL","Get accel data - BMA400_DATA_SENSOR_TIME\n");

    ESP_LOGI("ACCEL","Accel Gravity data in m/s^2\n");

    while (n_samples && (rslt == BMA400_OK))
    {
        rslt = bma400_get_interrupt_status(&int_status, _bma400_dev);
        bma400_check_rslt("bma400_get_interrupt_status", rslt);

        if (int_status & BMA400_ASSERTED_DRDY_INT)
        {
            rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &data, _bma400_dev);
            bma400_check_rslt("bma400_get_accel_data", rslt);

            // 12-bit accelerometer at range 2G /
            x = lsb_to_ms2(data.x, 2, 12);
            y = lsb_to_ms2(data.y, 2, 12);
            z = lsb_to_ms2(data.z, 2, 12);
            t = (float)data.sensortime * SENSOR_TICK_TO_S;

            ESP_LOGI("ACCEL","Gravity-x : %.2f,   Gravity-y : %.2f,  Gravity-z :  %.2f,   t(s) : %.4f\r\n", x, y, z, t);            
            n_samples--;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
} 

/* orient_feature interrupts */
static void accel_task (void *pvParameters)
{

    struct bma400_dev *dev = (struct bma400_dev*)pvParameters;

    uint16_t int_status = 0;
    uint8_t orient_cnter = 0;
    struct bma400_sensor_conf accel_settin[2];
    struct bma400_sensor_data accel;
    int8_t rslt = 0;

    accel_settin[0].type = BMA400_ORIENT_CHANGE_INT;
    accel_settin[1].type = BMA400_ACCEL;

    accel_settin[0].param.orient.axes_sel = 0;
    accel_settin[0].param.orient.data_src = 0;
    accel_settin[0].param.orient.int_chan = 0;
    accel_settin[0].param.orient.orient_int_dur = 0;
    accel_settin[0].param.orient.orient_thres = 0;
    accel_settin[0].param.orient.ref_update = 0;
    accel_settin[0].param.orient.stability_thres = 0;
    accel_settin[0].param.orient.orient_ref_x = 0;
    accel_settin[0].param.orient.orient_ref_y = 0;
    accel_settin[0].param.orient.orient_ref_z = 0;

    while(1)
    {

        rslt = bma400_get_sensor_conf(accel_settin, 1, dev);
        bma400_check_rslt("bma400_interface_init", rslt);

        rslt = bma400_get_interrupt_status(&int_status, dev);
        bma400_check_rslt("bma400_interface_init", rslt);

        //Orientación - Vuelco 
        if (int_status & BMA400_ASSERTED_ORIENT_CH)
        {
            ESP_LOGI("ACCEL","Orientation interrupt detected\n");

            if (orient_cnter == 0)
            {
                ESP_LOGD("ACCEL","Reference  : X: %d     Y : %d     Z : %d\n",
                        accel_settin[0].param.orient.orient_ref_x,
                        accel_settin[0].param.orient.orient_ref_y,
                        accel_settin[0].param.orient.orient_ref_z);
            }

            rslt = bma400_get_accel_data(BMA400_DATA_ONLY, &accel, dev);
            if (rslt == BMA400_OK)
            {
                ESP_LOGD("ACCEL","Accel data : X : %d   Y : %d    Z : %d\n", accel.x, accel.y, accel.z);
            }

            char inicio[50],envio[60];
            
            ArmadoTrInicio(inicio,AL_VUELCO);
            sprintf(envio,"%s,%s",inicio,TR_FIN);

            xSemaphoreTake(Mutex_UART_LTE,portMAX_DELAY);            
            Send_Transparent(envio,strlen(envio));
            xSemaphoreGive(Mutex_UART_LTE);

            uint8_t data=1;  //significa emergencia
            xQueueSendToBack(queue_rec_sd,&data,portMAX_DELAY);

        }
        //Sacudida eje X o eje Y
        if ((int_status & BMA400_ASSERTED_ACT_CH_X )  | (int_status & BMA400_ASSERTED_ACT_CH_Y))
        {
            ESP_LOGD("ACCEL","Activity change interrupt asserted on X or Y axis\n");

            rslt = bma400_get_accel_data(BMA400_DATA_SENSOR_TIME, &accel, _bma400_dev);
            bma400_check_rslt("bma400_get_accel_data_X", rslt);

            char inicio[50],envio[60];
            
            ArmadoTrInicio(inicio,AL_CHOQUE);
            sprintf(envio,"%s,%s",inicio,TR_FIN);

            xSemaphoreTake(Mutex_UART_LTE,portMAX_DELAY);            
            Send_Transparent(envio,strlen(envio));
            xSemaphoreGive(Mutex_UART_LTE);

            if (rslt == BMA400_OK)
            {
                ESP_LOGD("ACCEL","Accel Data :  X : %d    Y : %d    Z : %d    SENSOR_TIME : %d\n",
                        accel.x,
                        accel.y,
                        accel.z,
                        accel.sensortime);
            }

            uint8_t data=1;  //significa emergencia
            xQueueSendToBack(queue_rec_sd,&data,portMAX_DELAY);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}


static void accel_config_orientation(struct bma400_dev *dev)
{
    struct bma400_orient_int_conf test_orient_conf = { 0 };

    ESP_LOGD("ACCEL","Orient change interrupt with Z Axis enabled\n");
    test_orient_conf.axes_sel = BMA400_AXIS_Z_EN;
    test_orient_conf.data_src = BMA400_DATA_SRC_ACC_FILT2;
    test_orient_conf.orient_int_dur = 7; /* 10ms/LSB */
    test_orient_conf.orient_thres = TRESH_ORIENTATION;
    test_orient_conf.ref_update = BMA400_ORIENT_REFU_ACC_FILT_2;
    test_orient_conf.stability_thres = 10; /* 1 LSB = 8mg */

    int8_t rslt = 0;

    struct bma400_sensor_conf accel_settin[2];
    struct bma400_int_enable int_en[2];

    accel_settin[0].type = BMA400_ORIENT_CHANGE_INT;
    accel_settin[1].type = BMA400_ACCEL;

    rslt = bma400_get_sensor_conf(accel_settin, 2, dev);
    bma400_check_rslt("bma400_get_sensor_conf", rslt);

    accel_settin[0].param.orient.axes_sel = test_orient_conf.axes_sel;
    accel_settin[0].param.orient.data_src = test_orient_conf.data_src;
    accel_settin[0].param.orient.int_chan = test_orient_conf.int_chan;
    accel_settin[0].param.orient.orient_int_dur = test_orient_conf.orient_int_dur;
    accel_settin[0].param.orient.orient_thres = test_orient_conf.orient_thres;
    accel_settin[0].param.orient.ref_update = test_orient_conf.ref_update;
    accel_settin[0].param.orient.stability_thres = test_orient_conf.stability_thres;
    accel_settin[0].param.orient.orient_ref_x = test_orient_conf.orient_ref_x;
    accel_settin[0].param.orient.orient_ref_y = test_orient_conf.orient_ref_y;
    accel_settin[0].param.orient.orient_ref_z = test_orient_conf.orient_ref_z;

    accel_settin[1].param.accel.odr = BMA400_ODR_100HZ;
    accel_settin[1].param.accel.range = BMA400_RANGE_2G;
    accel_settin[1].param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;

    rslt = bma400_set_sensor_conf(accel_settin, 2, dev);
    bma400_check_rslt("bma400_set_sensor_conf", rslt);

    rslt = bma400_set_power_mode(BMA400_MODE_NORMAL, dev);
    bma400_check_rslt("bma400_set_power_mode", rslt);

    int_en[0].type = BMA400_ORIENT_CHANGE_INT_EN;
    int_en[0].conf = BMA400_ENABLE;

    int_en[1].type = BMA400_LATCH_INT_EN;
    int_en[1].conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(int_en, 2, dev);
    bma400_check_rslt("bma400_enable_interrupt", rslt);

}

static void accel_config_act_change(struct bma400_dev *dev)
{
    struct bma400_sensor_conf accel_settin[2] = { { 0 } };
    struct bma400_int_enable int_en;
    int8_t rslt = 0;

    accel_settin[0].type = BMA400_ACTIVITY_CHANGE_INT;
    accel_settin[1].type = BMA400_ACCEL;

    rslt = bma400_get_sensor_conf(accel_settin, 2, _bma400_dev);
    bma400_check_rslt("bma400_get_sensor_conf", rslt);

    accel_settin[0].param.act_ch.axes_sel = BMA400_AXIS_X_EN | BMA400_AXIS_Y_EN;
    accel_settin[0].param.act_ch.act_ch_ntps = BMA400_ACT_CH_SAMPLE_CNT_64;
    accel_settin[0].param.act_ch.data_source = BMA400_DATA_SRC_ACC_FILT1;
    accel_settin[0].param.act_ch.act_ch_thres = TRESH_ACT;

    /* Set the desired configurations to the sensor */
    rslt = bma400_set_sensor_conf(accel_settin, 1, _bma400_dev);//2
    bma400_check_rslt("bma400_set_sensor_conf", rslt);

    int_en.type = BMA400_ACTIVITY_CHANGE_INT_EN;
    int_en.conf = BMA400_ENABLE;

    rslt = bma400_enable_interrupt(&int_en, 1, _bma400_dev);
    bma400_check_rslt("bma400_enable_interrupt", rslt);
}

esp_err_t acelerometro_init()
{
    uint8_t rslt;
    esp_err_t ret;

    spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 2000000,
    .mode = 0,
    .spics_io_num = ACCEL_CS,
    .queue_size = 7,
    .flags = SPI_DEVICE_HALFDUPLEX
};

    // Inicialización del device en el SPI
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &accel_spi);

    if(ret != ESP_OK) {
        ESP_LOGE("SPI","Error al inicializar el Acelerometro");
        return ESP_FAIL;
    }
    ESP_LOGD("SPI","Bus Acelerometro inicializado");
    
    //Inicialización del BMA

    _bma400_dev=malloc(sizeof(struct bma400_dev));

    _bma400_dev->intf = BMA400_SPI_INTF;
    _bma400_dev->read = accel_read;
    _bma400_dev->write = accel_write;
    _bma400_dev->delay_us = delay_us;
    _bma400_dev->intf_ptr = accel_spi;

    rslt=bma400_init(_bma400_dev);
    bma400_check_rslt("bma400_init", rslt);

    rslt = bma400_soft_reset(_bma400_dev);
    bma400_check_rslt("bma400_soft_reset", rslt);

    ESP_LOGD("ACCEL","El numero del chip es %u",_bma400_dev->chip_id);

    //  Self test
    rslt = bma400_perform_self_test(_bma400_dev);
    bma400_check_rslt("bma400_perform_self_test",rslt);

    //  Configuración de las interrupciones
    accel_config_act_change(_bma400_dev);
    accel_config_orientation(_bma400_dev);

    ESP_LOGI("ACCEL","Creacion de la tarea");
    xTaskCreate(accel_task, "Acceleracion", 2048, _bma400_dev, tskIDLE_PRIORITY + 5UL, NULL);

    return ESP_OK;
}