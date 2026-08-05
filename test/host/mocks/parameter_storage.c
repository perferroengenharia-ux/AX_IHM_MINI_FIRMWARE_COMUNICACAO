#include "parameter_storage.h"

#include "mock_parameter_storage.h"

#include <string.h>

static bool s_has_blob;
static ihm_parameter_blob_t s_blob;
static bool s_has_motor_frequency;
static uint16_t s_motor_frequency;

void mock_parameter_storage_reset(void)
{
    s_has_blob = false;
    s_has_motor_frequency = false;
    s_motor_frequency = 0U;
    (void)memset(&s_blob, 0, sizeof(s_blob));
}

esp_err_t parameter_storage_load_motor_frequency(uint16_t *centihz)
{
    if (!s_has_motor_frequency || (centihz == NULL))
    {
        return ESP_FAIL;
    }
    *centihz = s_motor_frequency;
    return ESP_OK;
}

esp_err_t parameter_storage_save_motor_frequency(uint16_t centihz)
{
    s_motor_frequency = centihz;
    s_has_motor_frequency = true;
    return ESP_OK;
}

void mock_parameter_storage_set(const ihm_parameter_blob_t *parameters)
{
    s_blob = *parameters;
    s_has_blob = true;
}

void mock_parameter_storage_corrupt(void)
{
    s_blob.crc ^= 1U;
}

esp_err_t parameter_storage_save(const ihm_parameter_blob_t *parameters)
{
    if (!ihm_parameters_blob_is_valid(parameters))
    {
        return ESP_ERR_INVALID_ARG;
    }
    s_blob = *parameters;
    s_has_blob = true;
    return ESP_OK;
}

esp_err_t parameter_storage_init(ihm_parameter_blob_t *parameters,
                                 bool *defaults_loaded)
{
    if (s_has_blob && ihm_parameters_blob_is_valid(&s_blob))
    {
        *parameters = s_blob;
        *defaults_loaded = false;
    }
    else
    {
        ihm_parameters_load_defaults(parameters);
        *defaults_loaded = true;
        (void)parameter_storage_save(parameters);
    }
    return ESP_OK;
}
