#pragma once

#ifndef ESP_DS18B20_H
#define ESP_DS18B20_H

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "aquarium_board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_DS18B20_0 AQUARIUM_GPIO_DS18B20_0

void esp_ds18b20_task(void *pvParameters);
float esp_ds18b20_get_readings(int index);
int esp_ds18b20_get_device_count(void);

// Stuck detection: Get the time elapsed since the reading last changed for a sensor
// Returns: time in seconds since last reading change. Returns -1 if invalid index.
int64_t esp_ds18b20_get_stuck_duration_seconds(int index);

// Check if a sensor reading has been stuck for longer than the stuck threshold (2 hours)
// Returns: true if stuck, false otherwise or if invalid index
bool esp_ds18b20_is_reading_stuck(int index);

// Reset the stuck counter for a sensor (used after sensor re-initialization)
// Returns: ESP_OK on success, ESP_ERR_INVALID_ARG if invalid index
esp_err_t esp_ds18b20_reset_stuck_counter(int index);

#endif // ESP_DS18B20_H
#ifdef __cplusplus
}
#endif
