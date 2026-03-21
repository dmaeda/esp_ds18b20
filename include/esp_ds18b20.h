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

#endif // ESP_DS18B20_H
#ifdef __cplusplus
}
#endif
