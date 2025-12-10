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
const gpio_num_t LED_PIN_STANDBY = GPIO_NUM_25;  // Add this pin

#define BLINK_DELAY_MS 60000  // Auto-off after 60 seconds (main timer)
#define INACTIVITY_TIMEOUT_MS 10000  // Turn off after 20 seconds inactivity


typedef enum {
    BLINKS_OFF = 5,
    BLINKS_3 = 3,
    BLINKS_5 = 5,
    BLINKS_10 = 10
} num_signal_blinks_t;



bool state = false;
bool buttonPressed = false;
uint64_t ledOnTime = 0;

uint64_t buttonPressStartTime = 0;


// Track last activity
bool longPressDetected = false;
uint64_t lastActivityTime = 0; 

static const char *TAG = "LED_CONTROL";

uint64_t get_now_time() {
    uint64_t current_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    // ESP_LOGI(TAG, "Time: %llu", current_ms);
    fflush(stdout);
    return current_ms;
}

void checkInactivity(num_signal_blinks_t blink_count) {
    uint64_t currentTime = get_now_time();
    uint64_t inactiveTime = currentTime - lastActivityTime;
    
    // Check if approaching inactivity timeout
    if (state && inactiveTime >= (INACTIVITY_TIMEOUT_MS - 3000) && 
        inactiveTime < INACTIVITY_TIMEOUT_MS) {
        // Blink faster to indicate approaching timeout
        static uint64_t lastBlink = 0;
        if ((currentTime - lastBlink) > 200) {  // Fast blink (5 Hz)
            gpio_set_level(LED_PIN_BUTTON, !gpio_get_level(LED_PIN_BUTTON));
            lastBlink = currentTime;
            ESP_LOGI(TAG, "Inactivity warning: %llu ms remaining", 
                     INACTIVITY_TIMEOUT_MS - inactiveTime);
        }
    }
    
    // Turn off after 20 seconds of inactivity
    if (state && inactiveTime >= INACTIVITY_TIMEOUT_MS) {
        ESP_LOGI(TAG, "20 seconds inactivity - turning off LED");
        state = false;
        
        // Quick blink to indicate inactivity timeout
        for(int i = 0; i < 2; i++) {
            gpio_set_level(LED_PIN_BUTTON, 1);
            vTaskDelay(50 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN_BUTTON, 0);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        
        gpio_set_level(LED_PIN_RELAY, 0);
        ESP_LOGI(TAG, "LED turned OFF due to inactivity");
        gpio_set_level(LED_PIN_STANDBY, 1);
    }
}

void turnOff(num_signal_blinks_t blink_count) {
    uint64_t currentTime = get_now_time();

            // ESP_LOGI(TAG, "Current time: %d", currentTime);
    if (state && (currentTime - ledOnTime >= BLINK_DELAY_MS)) {
        state = false;
        for(int i = 0; i < blink_count; i++) {
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

    gpio_reset_pin(LED_PIN_STANDBY);
    gpio_set_direction(LED_PIN_STANDBY, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_STANDBY, 1);

    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "System started. Press button to toggle LED.");
    ESP_LOGI(TAG, "System started");
    ESP_LOGI(TAG, "Auto-off: %d seconds", BLINK_DELAY_MS / 1000);
    ESP_LOGI(TAG, "Inactivity timeout: %d seconds", INACTIVITY_TIMEOUT_MS / 1000);

        // Initialize last activity time
    lastActivityTime = get_now_time();

    while (1) {
        int buttonState = !gpio_get_level(BUTTON_PIN);
        // ESP_LOGI(TAG, "Button state is %d ", buttonState);

        // Button pressed = LOW (0) with pull-up
        if (buttonState == 0 && !buttonPressed) {
            gpio_set_level(LED_PIN_STANDBY, 0);
            // Update activity timestamp - reset inactivity timer
            lastActivityTime = get_now_time();
            state = !state;

            


            
            if (state) {

                gpio_set_level(LED_PIN_BUTTON, 1);
                gpio_set_level(LED_PIN_RELAY, 1);
                ledOnTime = get_now_time();
                ESP_LOGI(TAG, "LED ON. Timers: %ds auto-off, %ds inactivity", 
                         BLINK_DELAY_MS / 1000, INACTIVITY_TIMEOUT_MS / 1000);

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
        
        turnOff(BLINKS_OFF);
            buttonPressStartTime = get_now_time();

            // ESP_LOGI(TAG, "Time: %llu", buttonPressStartTime);

            // if(buttonPressStartTime > sleepTimer) {
            //     esp_deep_sleep_start();//Go to sleep

            // }
        checkInactivity(BLINKS_OFF);
        
        // If LED is on, show normal blinking pattern based on activity
        if (state) {
            // gpio_set_level(LED_PIN_STANDBY, 0);
            uint64_t currentTime = get_now_time();
            uint64_t inactiveTime = currentTime - lastActivityTime;
            
            // Normal slow blink when recently active
            static uint64_t lastNormalBlink = 0;
            if (inactiveTime < 10000) {  // Active in last 10 seconds
                if ((currentTime - lastNormalBlink) > 1000) {  // 1 Hz blink
                    gpio_set_level(LED_PIN_BUTTON, !gpio_get_level(LED_PIN_BUTTON));
                    lastNormalBlink = currentTime;
                    
                }
            } else {
                // Keep LED solid after 10 seconds of no button presses
                gpio_set_level(LED_PIN_BUTTON, 1);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

} // extern "C"