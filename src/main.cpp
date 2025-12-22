extern "C" {
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "Helper.h"
#include "ina219_driver.h"

// ==================== CONFIGURATION ====================

#define WIFI_SSID       "ELOPARWFNT"
#define WIFI_PASSWORD   "24680135790!!**"
#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define SHUNT_RESISTOR_OHMS 0.1f
#define MAX_EXPECTED_CURRENT 3.2f
#define CUTTER_THRESHOLD_MA 66.0f

// GPIO Pins
const gpio_num_t BUTTON_PIN = GPIO_NUM_15;
const gpio_num_t LED_PIN_CUTTER_YELLOW = GPIO_NUM_23;
const gpio_num_t LED_PIN_RELAY_RED = GPIO_NUM_13;
const gpio_num_t LED_PIN_WORK_GREEN = GPIO_NUM_32;
const gpio_num_t LED_PIN_STANDBY_RED = GPIO_NUM_25;
const gpio_num_t LED_PIN_WIFI_BLUE = GPIO_NUM_26;

// Timers
#define BLINK_DELAY_MS 60000
#define INACTIVITY_TIMEOUT_MS 60000

// ==================== GLOBAL VARIABLES ====================

typedef enum {
    BLINKS_OFF = 5,
    BLINKS_3 = 3,
    BLINKS_5 = 5,
    BLINKS_10 = 10,
    BLINKS_13 = 13
} num_signal_blinks_t;

void checkInactivity(num_signal_blinks_t blink_count);

bool state = false;
bool buttonPressed = false;
uint64_t ledOnTime = 0;
uint64_t buttonPressStartTime = 0;
bool longPressDetected = false;
uint64_t lastActivityTime = 0;
bool cutter_is_on = false;
bool wifi_connected = false;
char ip_address[16] = "0.0.0.0";

static const char *TAG = "MAIN";
static const char *WIFI_TAG = "WIFI";
static const char *OTA_TAG = "OTA";

ina219_dev_t ina219_dev;
static httpd_handle_t ota_server = NULL;

// ==================== HELPER FUNCTIONS ====================

uint64_t get_now_time_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000);
}

void cutterOn() {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN_CUTTER_YELLOW, 1);
    cutter_is_on = true;
    ESP_LOGI("CUTTER", "Cutter activated");

    vTaskDelay(1500 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN_CUTTER_YELLOW, 0);
    cutter_is_on = false;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI("CUTTER", "Cutter deactivated");
    gpio_set_level(LED_PIN_RELAY_RED, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN_WORK_GREEN, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(LED_PIN_STANDBY_RED, 1);
    state = false;
    buttonPressed = false;
    ESP_LOGI(TAG, "State: %s", state ? "ON" : "OFF");
    ESP_LOGI(TAG, "Button pressed: %s", buttonPressed ? "true" : "false");
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
    
    float shunt_mv = ina219_read_shunt_voltage(&ina219_dev);
    float expected_current_ma = (shunt_mv / 1000.0f) / SHUNT_RESISTOR_OHMS * 1000.0f;
    
    ESP_LOGI("INA219_CAL", "Shunt: %.2fmV -> Expected: %.1fmA", shunt_mv, expected_current_ma);
}

void test_ina219_with_load() {
    ESP_LOGI("TEST", "=== INA219 Load Test ===");
    
    gpio_set_level(LED_PIN_RELAY_RED, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    float i1 = ina219_read_current(&ina219_dev);
    float s1 = ina219_read_shunt_voltage(&ina219_dev);
    
    ESP_LOGI("TEST", "Turning relay ON for test...");
    gpio_set_level(LED_PIN_RELAY_RED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    float i2 = ina219_read_current(&ina219_dev);
    float s2 = ina219_read_shunt_voltage(&ina219_dev);
    
    gpio_set_level(LED_PIN_RELAY_RED, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    if (s2 > s1) {
        float shunt_diff_mv = s2 - s1;
        float expected_current_ma = (shunt_diff_mv / 1000.0f) / SHUNT_RESISTOR_OHMS * 1000.0f;
        ESP_LOGI("TEST", "Current increase: %.1fmA (from shunt: %.1fmA)", i2-i1, expected_current_ma);
    }
}

void read_power_measurements() {
    float current = ina219_read_current(&ina219_dev);
    float shunt = ina219_read_shunt_voltage(&ina219_dev);
    
    float shunt_current_ma = (shunt / 1000.0f) / SHUNT_RESISTOR_OHMS * 1000.0f;
    
    ESP_LOGI("POWER", "Current: %.1fmA (shunt: %.1fmA), Shunt: %.2fmV", 
             current, shunt_current_ma, shunt);
    
    if (current > 2000.0f || shunt_current_ma > 2000.0f) {
        ESP_LOGW("POWER", "! Overcurrent detected");
    }
}

void checkInactivity(num_signal_blinks_t blink_count) {
    uint64_t currentTime = get_now_time_ms();
    uint64_t inactiveTime = currentTime - lastActivityTime;
    
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
    
    if (state && inactiveTime >= INACTIVITY_TIMEOUT_MS) {
        ESP_LOGI(TAG, "Inactivity timeout - turning off");
        state = false;
        
        gpio_set_level(LED_PIN_WORK_GREEN, 0);
        gpio_set_level(LED_PIN_RELAY_RED, 1);
        
        for(int i = 0; i < 10; i++) {
            gpio_set_level(LED_PIN_WORK_GREEN, 1);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN_WORK_GREEN, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        gpio_set_level(LED_PIN_RELAY_RED, 0);
        gpio_set_level(LED_PIN_STANDBY_RED, 1);
        ESP_LOGI(TAG, "System turned OFF due to inactivity");
    }
}

// ==================== WIFI FUNCTIONS ====================

void wifi_init_simple(void) {
    ESP_LOGI(WIFI_TAG, "Starting WiFi for OTA...");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_LOGI(WIFI_TAG, "Connecting to: %s", WIFI_SSID);
    
    int retries = 0;
    while (retries < 20) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            wifi_connected = true;
            gpio_set_level(LED_PIN_WIFI_BLUE, 1);
            
            esp_netif_ip_info_t ip_info;
            esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                // FIXED: Use correct IP formatting
                esp_ip4addr_ntoa(&ip_info.ip, ip_address, sizeof(ip_address));
                ESP_LOGI(WIFI_TAG, "WiFi connected! IP: %s", ip_address);
            }
            break;
        }
        
        vTaskDelay(500 / portTICK_PERIOD_MS);
        retries++;
        gpio_set_level(LED_PIN_WIFI_BLUE, retries % 2);
    }
    
    if (!wifi_connected) {
        ESP_LOGW(WIFI_TAG, "WiFi connection failed");
        gpio_set_level(LED_PIN_WIFI_BLUE, 0);
    }
}

// ==================== OTA SERVER FUNCTIONS ====================

static esp_err_t ota_update_handler(httpd_req_t *req) {
    ESP_LOGI(OTA_TAG, "OTA update started");
    
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    
    if (update_partition == NULL) {
        ESP_LOGE(OTA_TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }
    
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "OTA begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }
    
    char ota_buffer[1024];
    int content_length = req->content_len;
    int received = 0;
    
    ESP_LOGI(OTA_TAG, "Receiving firmware, size: %d bytes", content_length);
    
    while (received < content_length) {
        int recv_len = httpd_req_recv(req, ota_buffer, sizeof(ota_buffer));
        if (recv_len < 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(OTA_TAG, "OTA receive error: %d", recv_len);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive error");
            return ESP_FAIL;
        }
        
        err = esp_ota_write(ota_handle, ota_buffer, recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(OTA_TAG, "OTA write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Write failed");
            return ESP_FAIL;
        }
        
        received += recv_len;
        if ((received % 4096) == 0 || received == content_length) {
            ESP_LOGI(OTA_TAG, "Received: %d/%d bytes (%.1f%%)", 
                     received, content_length, (received * 100.0) / content_length);
        }
    }
    
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(OTA_TAG, "Image validation failed, image is corrupted");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Image validation failed");
        } else {
            ESP_LOGE(OTA_TAG, "OTA end failed: %s", esp_err_to_name(err));
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        }
        return ESP_FAIL;
    }
    
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(OTA_TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot failed");
        return ESP_FAIL;
    }
    
    const char* resp = "OTA update successful! Rebooting...";
    httpd_resp_send(req, resp, strlen(resp));
    
    ESP_LOGI(OTA_TAG, "OTA update complete, restarting in 2 seconds...");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();
    
    return ESP_OK;
}

static esp_err_t test_get_handler(httpd_req_t *req) {
    char response[256];
    // FIXED: Properly format the IP address
    sprintf(response, "ESP32 OTA Server Ready\nUse: curl -X POST http://%s:3232 --data-binary @firmware.bin", ip_address);
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static void start_ota_server(void) {
    ESP_LOGI(OTA_TAG, "Starting OTA HTTP server on port 3232...");
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 3232;
    config.ctrl_port = 3232;
    config.max_open_sockets = 3;
    config.backlog_conn = 3;
    config.lru_purge_enable = true;
    
    if (httpd_start(&ota_server, &config) == ESP_OK) {
        ESP_LOGI(OTA_TAG, "OTA HTTP server started successfully!");
        ESP_LOGI(OTA_TAG, "Listening on: http://%s:3232", ip_address);
        ESP_LOGI(OTA_TAG, "Use: curl -X POST http://%s:3232 --data-binary @firmware.bin", ip_address);
        
        // Register OTA endpoint
        httpd_uri_t ota_uri = {
            .uri = "/",
            .method = HTTP_POST,
            .handler = ota_update_handler,
            .user_ctx = NULL
        };
        
        httpd_register_uri_handler(ota_server, &ota_uri);
        
        // Also register a simple GET endpoint for testing
        httpd_uri_t test_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = test_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(ota_server, &test_uri);
        
    } else {
        ESP_LOGE(OTA_TAG, "Failed to start OTA HTTP server");
    }
}

static void initialize_ota(void) {
    ESP_LOGI(OTA_TAG, "========================================");
    ESP_LOGI(OTA_TAG, "OTA Update System Initialized");
    ESP_LOGI(OTA_TAG, "IP: %s | Port: 3232", ip_address);
    ESP_LOGI(OTA_TAG, "========================================");
    
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running) {
        ESP_LOGI(OTA_TAG, "Running partition: %s", running->label);
    }
    
    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition) {
        ESP_LOGI(OTA_TAG, "OTA update partition: %s (size: %d bytes)", 
                 ota_partition->label, ota_partition->size);
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
    
    gpio_reset_pin(LED_PIN_WIFI_BLUE);
    gpio_set_direction(LED_PIN_WIFI_BLUE, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_WIFI_BLUE, 0);

    ESP_LOGI(TAG, "=== ESP32 Cutter System ===");
    ESP_LOGI(TAG, "Firmware v1.0.2 with OTA Support");
    ESP_LOGI(TAG, "Cutter threshold: %.0fmA", CUTTER_THRESHOLD_MA);

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

    // Initialize WiFi
    wifi_init_simple();
    
    // Start OTA server if WiFi connected
    if (wifi_connected) {
        initialize_ota();
        start_ota_server();
    } else {
        ESP_LOGW(TAG, "OTA disabled - WiFi not connected");
    }

    lastActivityTime = get_now_time_ms();
    uint32_t last_power_read = 0;
    uint32_t last_status_log = 0;
    float last_logged_current = 0;
    
    ESP_LOGI(TAG, "System ready. Press button to toggle system.");
    if (wifi_connected) {
        ESP_LOGI(TAG, "OTA available at: http://%s:3232", ip_address);
    }
    
    while (1) {
        // Button handling
        int buttonState = !gpio_get_level(BUTTON_PIN);
        
        if (buttonState == 0 && !buttonPressed) {
            gpio_set_level(LED_PIN_STANDBY_RED, 0);
            lastActivityTime = get_now_time_ms();
            state = !state;
            
            if (state) {
                gpio_set_level(LED_PIN_WORK_GREEN, 1);
                gpio_set_level(LED_PIN_RELAY_RED, 1);
                
                vTaskDelay(500 / portTICK_PERIOD_MS);
                
                float current = ina219_read_current(&ina219_dev);
                ESP_LOGI("CURRENT", "Initial current after relay ON: %.1fmA", current);
                
                if (current > CUTTER_THRESHOLD_MA) {
                    ESP_LOGI("CUTTER", "Current > %.0fmA - Starting cutter", CUTTER_THRESHOLD_MA);
                    cutterOn();
                } else {
                    ESP_LOGI("CUTTER", "Current ≤ %.0fmA - Cutter remains OFF", CUTTER_THRESHOLD_MA);
                }
                
                ledOnTime = get_now_time_ms();
                ESP_LOGI(TAG, "System ON. Current: %.1fmA", current);
                
            } else {
                gpio_set_level(LED_PIN_WORK_GREEN, 0);
                gpio_set_level(LED_PIN_RELAY_RED, 0);
                gpio_set_level(LED_PIN_STANDBY_RED, 1);
                ESP_LOGI(TAG, "System OFF");
            }
            
            ESP_LOGI(TAG, "State: %s", state ? "ON" : "OFF");
            buttonPressed = true;
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        
        if (buttonState == 1 && buttonPressed) {
            buttonPressed = false;
        }
        
        // Continuous current monitoring
        if (state) {
            float current = ina219_read_current(&ina219_dev);
            
            if (current > CUTTER_THRESHOLD_MA && !cutter_is_on) {
                ESP_LOGI("CUTTER", "Current increased > %.0fmA - Starting cutter", CUTTER_THRESHOLD_MA);
                cutterOn();
            }
            
            if (get_now_time_ms() - last_power_read > 2000 || 
                fabs(current - last_logged_current) > 5.0f) {
                ESP_LOGI("MONITOR", "Current: %.1fmA, Cutter: %s", 
                         current, cutter_is_on ? "ON" : "OFF");
                last_logged_current = current;
                last_power_read = get_now_time_ms();
            }
        }
        
        // Check inactivity timeout
        checkInactivity(BLINKS_OFF);
        
        // Log status every 30 seconds
        if (get_now_time_ms() - last_status_log > 30000) {
            ESP_LOGI(TAG, "Status: %s | WiFi: %s | Uptime: %llus", 
                     state ? "ON" : "OFF", 
                     wifi_connected ? ip_address : "OFF",
                     get_now_time_ms() / 1000);
            last_status_log = get_now_time_ms();
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

} // extern "C"