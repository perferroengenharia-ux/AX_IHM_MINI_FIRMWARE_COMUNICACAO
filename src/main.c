/**
 * @file main.c
 * @brief Ponto de entrada do firmware ESP32-S3.
 */

#include "app.h"
#include "remote_network.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(app_start());
    const esp_err_t remote_error = remote_network_start();
    if (remote_error != ESP_OK)
    {
        /* A falha da interface remota nao pode interromper o Modbus validado. */
        ESP_LOGE(TAG, "Acesso MQTT/AP indisponivel: %s",
                 esp_err_to_name(remote_error));
    }
}
