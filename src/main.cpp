extern "C" {
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "Helper.h"
#include "ina219_driver.h"

// ==================== CONFIGURATION ====================

// I2C Configuration for INA219
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// INA219 Calibration
#define SHUNT_RESISTOR_OHMS 0.1f      // 0.1 ohm shunt resistor (most common)
#define MAX_EXPECTED_CURRENT 3.2f     // 3.2A max for 0.1 ohm shunt

// GPIO Pins
const gpio_num_t BUTTON_PIN = GPIO_NUM_15;
const gpio_num_t LED_PIN_CUTTER_YELLOW = GPIO_NUM_23;
const gpio_num_t LED_PIN_RELAY_RED = GPIO_NUM_13;
const gpio_num_t LED_PIN_WORK_GREEN = GPIO_NUM_32;
const gpio_num_t LED_PIN_STANDBY_RED = GPIO_NUM_25;

// Timers
#define BLINK_DELAY_MS 60000          // Auto-off after 60 seconds
#define INACTIVITY_TIMEOUT_MS 20000*3   // Turn off after 20 seconds inactivity

// ==================== GLOBAL VARIABLES ====================

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
bool longPressDetected = false;
uint64_t lastActivityTime = 0;

static const char *TAG = "MAIN";
ina219_dev_t ina219_dev;

// ==================== HELPER FUNCTIONS ====================

uint64_t get_now_time_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000);
}

void cutterOn() {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN_CUTTER_YELLOW, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN_CUTTER_YELLOW, 0);
}

void check_ina219_config() {
    uint16_t config = 0;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, INA219_REG_CONFIG, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    uint8_t data[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    config = (data[0] << 8) | data[1];
    
    ESP_LOGI("INA219_CFG", "Config Register: 0x%04X", config);
    ESP_LOGI("INA219_CFG", "  Bus Voltage Range: %s", (config & (1 << 13)) ? "32V" : "16V");
    ESP_LOGI("INA219_CFG", "  PGA Gain: %d", (config >> 11) & 0x03);
    ESP_LOGI("INA219_CFG", "  Mode: %s", 
             (config & 0x7) == 0x7 ? "Continuous" : 
             (config & 0x7) == 0x5 ? "Triggered" : "Shutdown");
}

void debug_ina219_calibration() {
    uint16_t cal_reg = 0;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, INA219_REG_CALIBRATION, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    uint8_t data[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    cal_reg = (data[0] << 8) | data[1];
    
    ESP_LOGI("INA219_CAL", "Calibration Register: 0x%04X (%u)", cal_reg, cal_reg);
    
    // Read shunt and calculate expected current
    float shunt_mv = ina219_read_shunt_voltage(&ina219_dev);
    float expected_current_ma = (shunt_mv / 1000.0f) / SHUNT_RESISTOR_OHMS * 1000.0f;
    
    ESP_LOGI("INA219_CAL", "Shunt: %.2fmV -> Expected: %.1fmA", shunt_mv, expected_current_ma);
}

void test_ina219_with_load() {
    ESP_LOGI("TEST", "=== INA219 Load Test ===");
    
    // Test 1: No load (relay off)
    gpio_set_level(LED_PIN_RELAY_RED, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    float v1 = ina219_read_bus_voltage(&ina219_dev);
    float i1 = ina219_read_current(&ina219_dev);
    float s1 = ina219_read_shunt_voltage(&ina219_dev);
    
    // Test 2: With load (relay on)
    ESP_LOGI("TEST", "Turning relay ON for test...");
    gpio_set_level(LED_PIN_RELAY_RED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    float v2 = ina219_read_bus_voltage(&ina219_dev);
    float i2 = ina219_read_current(&ina219_dev);
    float s2 = ina219_read_shunt_voltage(&ina219_dev);
    
    // Turn relay back off
    gpio_set_level(LED_PIN_RELAY_RED, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    ESP_LOGI("TEST", "OFF: V=%.2fV, I=%.2fmA, Shunt=%.2fmV", v1, i1, s1);
    ESP_LOGI("TEST", "ON:  V=%.2fV, I=%.2fmA, Shunt=%.2fmV", v2, i2, s2);
    ESP_LOGI("TEST", "DIFF: ΔV=%.2fV, ΔI=%.2fmA, ΔShunt=%.2fmV", 
             v2-v1, i2-i1, s2-s1);
    
    // Calculate expected current from shunt voltage
    if (s2 > s1) {
        float shunt_diff_mv = s2 - s1;
        float expected_current_ma = (shunt_diff_mv / 1000.0f) / SHUNT_RESISTOR_OHMS * 1000.0f;
        ESP_LOGI("TEST", "Expected current from shunt: %.1fmA", expected_current_ma);
        ESP_LOGI("TEST", "Measured current: %.1fmA", i2-i1);
    }
}

void read_power_measurements() {
    float voltage = ina219_read_bus_voltage(&ina219_dev);
    float current = ina219_read_current(&ina219_dev);
    float power = ina219_read_power(&ina219_dev);
    float shunt = ina219_read_shunt_voltage(&ina219_dev);
    
    // Always calculate current from shunt as backup
    float shunt_current_ma = (shunt / 1000.0f) / SHUNT_RESISTOR_OHMS * 1000.0f;
    
    ESP_LOGI("POWER", "V: %.2fV, I: %.1fmA (shunt: %.1fmA), P: %.1fmW, Shunt: %.2fmV", 
             voltage, current, shunt_current_ma, power, shunt);
    
    if (current > 2000.0f || shunt_current_ma > 2000.0f) {
        ESP_LOGW("POWER", "! Overcurrent detected");
    }
}

void checkInactivity(num_signal_blinks_t blink_count) {
    uint64_t currentTime = Helper::get_now_time();
    uint64_t inactiveTime = currentTime - lastActivityTime;
    
    // Check if approaching inactivity timeout
    if (state && inactiveTime >= (INACTIVITY_TIMEOUT_MS - 3000) && 
        inactiveTime < INACTIVITY_TIMEOUT_MS) {
        static uint64_t lastBlink = 0;
        if ((currentTime - lastBlink) > 200) {
            gpio_set_level(LED_PIN_WORK_GREEN, !gpio_get_level(LED_PIN_WORK_GREEN));
            lastBlink = currentTime;
            ESP_LOGI(TAG, "Inactivity warning: %llu ms remaining", 
                     INACTIVITY_TIMEOUT_MS - inactiveTime);
        }
    }
    
    // Turn off after inactivity timeout
    if (state && inactiveTime >= INACTIVITY_TIMEOUT_MS) {
        ESP_LOGI(TAG, "Inactivity timeout - turning off");
        state = false;
        
        gpio_set_level(LED_PIN_WORK_GREEN, 0);
        gpio_set_level(LED_PIN_RELAY_RED, 0); 
        
        // Blink standby LED 5 times
        for(int i = 0; i < 5; i++) {
            gpio_set_level(LED_PIN_STANDBY_RED, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN_STANDBY_RED, 0);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        
        gpio_set_level(LED_PIN_STANDBY_RED, 1);
        ESP_LOGI(TAG, "System turned OFF due to inactivity");
    }
}

// ==================== MAIN APPLICATION ====================

void app_main(void) {
    // Initialize GPIO
    gpio_reset_pin(LED_PIN_RELAY_RED);
    gpio_set_direction(LED_PIN_RELAY_RED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_RELAY_RED, 0);

    gpio_reset_pin(LED_PIN_WORK_GREEN);
    gpio_set_direction(LED_PIN_WORK_GREEN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_WORK_GREEN, 0);

    gpio_reset_pin(LED_PIN_STANDBY_RED);
    gpio_set_direction(LED_PIN_STANDBY_RED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_STANDBY_RED, 1);

    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    gpio_reset_pin(LED_PIN_CUTTER_YELLOW);
    gpio_set_direction(LED_PIN_CUTTER_YELLOW, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_CUTTER_YELLOW, 0);

    ESP_LOGI(TAG, "System starting...");
    ESP_LOGI(TAG, "Auto-off: %d seconds", BLINK_DELAY_MS / 1000);
    ESP_LOGI(TAG, "Inactivity timeout: %d seconds", INACTIVITY_TIMEOUT_MS / 1000);

    // Initialize INA219
    ESP_LOGI(TAG, "Initializing INA219 sensor...");
    if (!ina219_init(&ina219_dev, I2C_PORT, INA219_ADDR, I2C_SDA_PIN, I2C_SCL_PIN,
                     SHUNT_RESISTOR_OHMS, MAX_EXPECTED_CURRENT)) {
        ESP_LOGE(TAG, "Failed to initialize INA219 - continuing without power monitoring");
    } else {
        ESP_LOGI(TAG, "INA219 initialized successfully");
        check_ina219_config();
        debug_ina219_calibration();
        test_ina219_with_load();
    }

    lastActivityTime = Helper::get_now_time();
    uint32_t last_power_read = 0;
    uint32_t last_status_log = 0;
    
    ESP_LOGI(TAG, "System ready. Press button to toggle cutter.");
    
    while (1) {
        // Button handling
        int buttonState = !gpio_get_level(BUTTON_PIN);
        
        if (buttonState == 0 && !buttonPressed) {
            gpio_set_level(LED_PIN_STANDBY_RED, 0);
            lastActivityTime = Helper::get_now_time();
            state = !state;
            
            if (state) {
                gpio_set_level(LED_PIN_WORK_GREEN, 1);
                gpio_set_level(LED_PIN_RELAY_RED, 1);
                cutterOn();
                ledOnTime = Helper::get_now_time();
                ESP_LOGI(TAG, "Cutter ON. Timers: %ds auto-off, %ds inactivity", 
                         BLINK_DELAY_MS / 1000, INACTIVITY_TIMEOUT_MS / 1000);
            } else {
                gpio_set_level(LED_PIN_WORK_GREEN, 0);
                gpio_set_level(LED_PIN_RELAY_RED, 0);
                gpio_set_level(LED_PIN_STANDBY_RED, 1);
                ESP_LOGI(TAG, "Cutter turned OFF manually.");
            }
            
            ESP_LOGI(TAG, "State: %s", state ? "ON" : "OFF");
            buttonPressed = true;
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        
        if (buttonState == 1 && buttonPressed) {
            buttonPressed = false;
            ESP_LOGI(TAG, "Button released");
        }
        
        // Check inactivity timeout
        checkInactivity(BLINKS_OFF);
        
        // Read power measurements every 2 seconds when cutter is on
        if (state && (get_now_time_ms() - last_power_read > 2000)) {
            read_power_measurements();
            last_power_read = get_now_time_ms();
        }
        
        // Log status every 30 seconds
        if (get_now_time_ms() - last_status_log > 30000) {
            ESP_LOGI(TAG, "System status: %s, Uptime: %llu seconds", 
                     state ? "ON" : "OFF", get_now_time_ms() / 1000);
            last_status_log = get_now_time_ms();
        }
   
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

} // extern "C"