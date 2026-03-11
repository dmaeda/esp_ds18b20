#pragma once

#ifndef ESP_DS18B20_H
#define ESP_DS18B20_H

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_IDF_TARGET_ESP32
    #define GPIO_DS18B20_0 (4)    
#elif CONFIG_IDF_TARGET_ESP32C3
    #define GPIO_DS18B20_0 (5)
#else
    #define GPIO_DS18B20_0 (4)    
#endif



void esp_ds18b20_task(void *pvParameters);
float esp_ds18b20_get_readings(int index);

#endif // ESP_DS18B20_H
#ifdef __cplusplus
}
#endif
