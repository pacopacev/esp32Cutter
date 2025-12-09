#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN 32  // Your LED on GPIO 32
#define BLINK_DELAY_MS 3000

static const char *TAG = "LED_BLINK";

void app_main(void)
{
    // Configure LED pin
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    ESP_LOGI(TAG, "Starting LED blink on GPIO %d", LED_PIN);
    ESP_LOGI(TAG, "Delay: %d ms", BLINK_DELAY_MS);
    
    while (1) {
        // Turn LED ON
        gpio_set_level(LED_PIN, 1);
        ESP_LOGI(TAG, "LED ON");
        vTaskDelay(BLINK_DELAY_MS / portTICK_PERIOD_MS);
        
        // Turn LED OFF
        gpio_set_level(LED_PIN, 0);
        ESP_LOGI(TAG, "LED OFF");
        vTaskDelay(BLINK_DELAY_MS / portTICK_PERIOD_MS);
    }
}