#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/task.h"

void fatal_error (void)
{
  ESP_LOGE("FATAL","El dispositivo se reiniciara en 5 seg");
  for(int i=0; i<5 ; i++)
    vTaskDelay(1000/portTICK_PERIOD_MS);
  esp_restart();
}