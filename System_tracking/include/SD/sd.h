esp_err_t sd_init_sdmode();
void sd_write(void *pvParameter);
void sd_read(void *pvParameter);
void create_sd_tasks(void);

/***DEFINES***********************************************************/

#define SD_DATA0    2
#define SD_CMD      15