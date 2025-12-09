extern "C" {

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

const gpio_num_t LED_PIN_BUTTON = GPIO_NUM_22;
const gpio_num_t LED_PIN_RELAY = GPIO_NUM_32;
const gpio_num_t BUTTON_PIN = GPIO_NUM_15;

#define BLINK_DELAY_MS 5000

bool state = false;
bool buttonPressed = false;
uint64_t ledOnTime = 0;

static const char *TAG = "LED_CONTROL";

uint64_t get_now_time() {
    uint64_t current_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    ESP_LOGI(TAG, "Time: %llu", current_ms);
    fflush(stdout);
    return current_ms;
}

void turnOff() {
    uint64_t currentTime = get_now_time();

            // ESP_LOGI(TAG, "Current time: %d", currentTime);
    if (state && (currentTime - ledOnTime >= BLINK_DELAY_MS)) {
        state = false;
        for(int i = 0; i < 5; i++) {
            gpio_set_level(LED_PIN_BUTTON, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN_BUTTON, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        gpio_set_level(LED_PIN_RELAY, 0);
        ESP_LOGI(TAG, "Auto-off: LED turned OFF after %d ms", BLINK_DELAY_MS);
    }
}

void app_main(void) {
    gpio_reset_pin(LED_PIN_RELAY);
    gpio_set_direction(LED_PIN_RELAY, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_RELAY, 0);

    gpio_reset_pin(LED_PIN_BUTTON);
    gpio_set_direction(LED_PIN_BUTTON, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_BUTTON, 0);

    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "System started. Press button to toggle LED.");

    while (1) {
        int buttonState = gpio_get_level(BUTTON_PIN);

        // Button pressed = LOW (0) with pull-up
        if (buttonState == 1 && !buttonPressed) {
            state = !state;
            
            if (state) {
                gpio_set_level(LED_PIN_BUTTON, 1);
                gpio_set_level(LED_PIN_RELAY, 1);
                ledOnTime = get_now_time();
                ESP_LOGI(TAG, "LED turned ON. Auto-off in %d ms", BLINK_DELAY_MS);
            } else {
                gpio_set_level(LED_PIN_BUTTON, 0);
                gpio_set_level(LED_PIN_RELAY, 0);
                ESP_LOGI(TAG, "LED turned OFF manually.");
            }
            
            ESP_LOGI(TAG, "Current state: %s", state ? "ON" : "OFF");
            buttonPressed = true;
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        
        // Button released = HIGH (1)
        if (buttonState == 1 && buttonPressed) {
            buttonPressed = false;
            ESP_LOGI(TAG, "Button released");
        }
        
        turnOff();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

} // extern "C"