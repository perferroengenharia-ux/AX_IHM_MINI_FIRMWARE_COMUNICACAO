#include "parameter_storage.h"

#include "nvs.h"
#include "nvs_flash.h"

#include <stddef.h>

#define PARAMETER_NVS_NAMESPACE "ihm_config"
#define PARAMETER_NVS_KEY       "parameters"

static nvs_handle_t s_nvs_handle;
static bool s_initialized;

esp_err_t parameter_storage_save(const ihm_parameter_blob_t *parameters)
{
    esp_err_t error;

    if (!s_initialized || !ihm_parameters_blob_is_valid(parameters))
    {
        return ESP_ERR_INVALID_STATE;
    }
    error = nvs_set_blob(s_nvs_handle,
                         PARAMETER_NVS_KEY,
                         parameters,
                         sizeof(*parameters));
    if (error == ESP_OK)
    {
        error = nvs_commit(s_nvs_handle);
    }
    return error;
}

esp_err_t parameter_storage_init(ihm_parameter_blob_t *parameters,
                                 bool *defaults_loaded)
{
    esp_err_t error;
    size_t stored_size = sizeof(*parameters);

    if ((parameters == NULL) || (defaults_loaded == NULL))
    {
        return ESP_ERR_INVALID_ARG;
    }

    error = nvs_flash_init();
    if ((error == ESP_ERR_NVS_NO_FREE_PAGES) ||
        (error == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        error = nvs_flash_erase();
        if (error == ESP_OK)
        {
            error = nvs_flash_init();
        }
    }
    if (error != ESP_OK)
    {
        return error;
    }

    error = nvs_open(PARAMETER_NVS_NAMESPACE,
                     NVS_READWRITE,
                     &s_nvs_handle);
    if (error != ESP_OK)
    {
        return error;
    }
    s_initialized = true;

    error = nvs_get_blob(s_nvs_handle,
                         PARAMETER_NVS_KEY,
                         parameters,
                         &stored_size);
    if ((error == ESP_OK) &&
        (stored_size == sizeof(*parameters)) &&
        ihm_parameters_blob_is_valid(parameters))
    {
        *defaults_loaded = false;
        return ESP_OK;
    }

    ihm_parameters_load_defaults(parameters);
    *defaults_loaded = true;
    return parameter_storage_save(parameters);
}
