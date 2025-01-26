#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Define the GPIO pin for the LED
#define LED_GPIO_PIN GPIO_NUM_2

void app_main(void)
{
    // Configure the GPIO pin as output
    gpio_reset_pin(LED_GPIO_PIN);
    gpio_set_direction(LED_GPIO_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        // Turn the LED on
        gpio_set_level(LED_GPIO_PIN, 1);
        printf("LED ON\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second

        // Turn the LED off
        gpio_set_level(LED_GPIO_PIN, 0);
        printf("LED OFF\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}