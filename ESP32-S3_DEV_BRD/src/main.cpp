#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "system.h"

extern "C" void app_main() {
  init();

  ESP_LOGI("APP_MAIN", "Starting application");

  while (1) {
    ESP_LOGI("APP_MAIN", "Running FreeRTOS task loop");
    vTaskDelay(pdMS_TO_TICKS(1000));  // 1-second delay
  }
}


