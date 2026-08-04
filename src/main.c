/**
 * @file main.c
 * @brief Ponto de entrada do firmware ESP32-S3.
 */

#include "app.h"

#include "esp_err.h"

void app_main(void)
{
    ESP_ERROR_CHECK(app_start());
}
