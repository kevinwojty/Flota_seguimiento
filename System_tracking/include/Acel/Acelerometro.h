#include "bma400.h"
#include "esp_err.h"

/* 39.0625us per tick */
#define SENSOR_TICK_TO_S  (0.0000390625f)
/* Earth's gravity in m/s^2 */
#define GRAVITY_EARTH     (9.80665f)
#define TRESH_ACT   45     /* 1 LSB = 8mg Treshold de cambio brusco de aceleración*/
#define TRESH_ORIENTATION   60 /* 1 LSB = 8mg */

#define ACCEL_CS   16

BMA400_INTF_RET_TYPE accel_read (uint8_t reg_addr, uint8_t *reg_data, uint32_t length,void *intf_ptr);
BMA400_INTF_RET_TYPE accel_write (uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,void *intf_ptr);
void delay_us (uint32_t period, void *intf_ptr);
esp_err_t acelerometro_init();
