extern "C" {
#include "Helper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


uint64_t Helper::get_now_time() {
    uint64_t current_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    // ESP_LOGI(TAG, "Time: %llu", current_ms);
    fflush(stdout);
    return current_ms;
}
}